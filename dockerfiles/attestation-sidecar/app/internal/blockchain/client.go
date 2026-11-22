package blockchain

import (
	"bytes"
	"context"
	"crypto/ecdsa"
	"encoding/hex"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"math/big"
	"net/http"
	"strings"
	"sync"
	"sync/atomic"
	"time"

	"app/internal/logger"
	// types.go is in the same package 'blockchain'
	"github.com/ethereum/go-ethereum/accounts/abi"
	"github.com/ethereum/go-ethereum/common"
	"github.com/ethereum/go-ethereum/common/hexutil"
	gethtypes "github.com/ethereum/go-ethereum/core/types"
	"github.com/ethereum/go-ethereum/crypto"
	"github.com/ethereum/go-ethereum/ethclient"
	"github.com/ethereum/go-ethereum/rpc"
)

// Pooled HTTP client for RPC calls
var pooledHTTPClient = &http.Client{
	Transport: &http.Transport{
		MaxIdleConns:        100,
		MaxIdleConnsPerHost: 100,
		IdleConnTimeout:     90 * time.Second,
	},
	Timeout: 10 * time.Second,
}

type BlockchainClient struct {
	client          *ethclient.Client
	privateKey      *ecdsa.PrivateKey
	ethAddress      common.Address
	contractAddress common.Address
	contractABI     abi.ABI
	chainID         *big.Int
	gasPrice        *big.Int // read once: fixed on this network (zero base fee)
	ethNodeURL      string

	nonceLock  sync.Mutex
	localNonce uint64

	heads *heads // block notifications, see FollowHeads

	// Settings fixed at deployment, read once (V3).
	juryTimeout atomic.Uint64
	freshness   atomic.Uint64

	agentNames sync.Map // address -> registered name, for the logs
}

// Helper struct for loading ABI JSON (Hardhat-style artifact: {"abi":[...]} )
type ABIFile struct {
	ABI json.RawMessage `json:"abi"`
}

// NewBlockchainClient initializes the connection and contract binding.
// Updated to remove redundant abiPath arg to match main.go
func NewBlockchainClient(ethAddressStr, privateKeyHex, ethNodeURL, contractAddrStr string, contractABI interface{}) (*BlockchainClient, error) {
	// 1. Connect
	// The pooled client keeps the connections to the node open: the prover,
	// the watcher and the juror of a sidecar query it at the same block.
	rpcClient, err := rpc.DialOptions(context.Background(), ethNodeURL, rpc.WithHTTPClient(pooledHTTPClient))
	if err != nil {
		return nil, fmt.Errorf("failed to connect to eth node: %v", err)
	}
	client := ethclient.NewClient(rpcClient)

	// 2. Private Key
	privateKeyHex = strings.TrimPrefix(privateKeyHex, "0x")
	privateKey, err := crypto.HexToECDSA(privateKeyHex)
	if err != nil {
		return nil, fmt.Errorf("invalid private key: %v", err)
	}

	// 3. Address
	publicKey := privateKey.Public()
	publicKeyECDSA, ok := publicKey.(*ecdsa.PublicKey)
	if !ok {
		return nil, errors.New("error casting public key to ECDSA")
	}
	derivedAddr := crypto.PubkeyToAddress(*publicKeyECDSA)

	if ethAddressStr != "" {
		provided := common.HexToAddress(ethAddressStr)
		if provided != derivedAddr {
			logger.Warn("Provided ETH_ADDRESS (%s) != address from private key (%s). Using derived.", provided.Hex(), derivedAddr.Hex())
		}
	}

	// 4. Load ABI (Logic simplified to use the interface directly)
	if contractABI == nil {
		return nil, errors.New("no ABI provided")
	}

	jsonBytes, err := json.Marshal(contractABI)
	if err != nil {
		return nil, fmt.Errorf("failed to marshal contract ABI: %v", err)
	}

	parsedABI, err := abi.JSON(strings.NewReader(string(jsonBytes)))
	if err != nil {
		return nil, fmt.Errorf("failed to parse ABI: %v", err)
	}

	// 5. Chain ID & Nonce
	ctx := context.Background()
	chainID, err := client.ChainID(ctx)
	if err != nil {
		return nil, fmt.Errorf("failed to get chainID: %v", err)
	}
	nonce, err := client.PendingNonceAt(ctx, derivedAddr)
	if err != nil {
		return nil, fmt.Errorf("failed to get nonce: %v", err)
	}
	gasPrice, err := client.SuggestGasPrice(ctx)
	if err != nil {
		return nil, fmt.Errorf("failed to get gas price: %v", err)
	}

	bc := &BlockchainClient{
		client:          client,
		privateKey:      privateKey,
		ethAddress:      derivedAddr,
		contractAddress: common.HexToAddress(contractAddrStr),
		contractABI:     parsedABI,
		chainID:         chainID,
		gasPrice:        gasPrice,
		ethNodeURL:      ethNodeURL,
		localNonce:      nonce,
		heads:           newHeads(),
	}

	logger.Info("Web3 Client Initialized:")
	logger.Info("Account Address  : %s", derivedAddr.Hex())
	logger.Info("Contract Address : %s", contractAddrStr)
	logger.Info("Node URL         : %s", ethNodeURL)

	return bc, nil
}

// -------------------------------------------------------------------------
// Core Helpers (Raw Calls & Manual Transactions)
// -------------------------------------------------------------------------

type rpcRequest struct {
	Jsonrpc string        `json:"jsonrpc"`
	Method  string        `json:"method"`
	Params  []interface{} `json:"params"`
	ID      int           `json:"id"`
}

type rpcError struct {
	Code    int    `json:"code"`
	Message string `json:"message"`
	Data    string `json:"data"` // revert data, when the call reverted
}

type rpcResponse struct {
	Result string    `json:"result"`
	Error  *rpcError `json:"error"`
}

// rawEthCall executes eth_call via direct JSON-RPC, from this agent's
// address: the contract checks the caller of some reads.
func (bc *BlockchainClient) rawEthCall(data []byte) ([]byte, error) {
	type callParams struct {
		From string `json:"from"`
		To   string `json:"to"`
		Data string `json:"data"`
	}

	params := callParams{
		From: bc.ethAddress.Hex(),
		To:   bc.contractAddress.Hex(),
		Data: "0x" + hex.EncodeToString(data),
	}

	req := rpcRequest{
		Jsonrpc: "2.0",
		Method:  "eth_call",
		Params:  []interface{}{params, "latest"},
		ID:      1,
	}

	reqBody, err := json.Marshal(req)
	if err != nil {
		return nil, err
	}

	resp, err := pooledHTTPClient.Post(bc.ethNodeURL, "application/json", bytes.NewReader(reqBody))
	if err != nil {
		return nil, err
	}
	defer resp.Body.Close()

	body, err := io.ReadAll(resp.Body)
	if err != nil {
		return nil, err
	}
	if resp.StatusCode < 200 || resp.StatusCode >= 300 {
		return nil, fmt.Errorf("RPC HTTP %d: %s", resp.StatusCode, string(body))
	}

	var rpcResp rpcResponse
	if err := json.Unmarshal(body, &rpcResp); err != nil {
		return nil, err
	}
	if rpcResp.Error != nil {
		if data, err := hexutil.Decode(rpcResp.Error.Data); err == nil {
			if reason, err := abi.UnpackRevert(data); err == nil {
				return nil, &RevertError{Reason: reason}
			}
		}
		return nil, fmt.Errorf("RPC error: %s", rpcResp.Error.Message)
	}

	resultHex := strings.TrimPrefix(rpcResp.Result, "0x")
	if resultHex == "" {
		return []byte{}, nil
	}
	return hex.DecodeString(resultHex)
}

func (bc *BlockchainClient) refreshNonce(ctx context.Context) error {
	n, err := bc.client.PendingNonceAt(ctx, bc.ethAddress)
	if err != nil {
		return err
	}
	bc.localNonce = n
	return nil
}

// RevertError is a transaction the contract would revert, with its reason.
type RevertError struct{ Reason string }

func (e *RevertError) Error() string { return "reverted: " + e.Reason }

// IsRevert tells whether err is a revert whose reason contains part.
func IsRevert(err error, part string) bool {
	var re *RevertError
	return errors.As(err, &re) && strings.Contains(re.Reason, part)
}

// revertReason turns a node error carrying revert data into a *RevertError.
func revertReason(err error) error {
	var dataErr rpc.DataError
	if errors.As(err, &dataErr) {
		if hexData, ok := dataErr.ErrorData().(string); ok {
			if data, derr := hexutil.Decode(hexData); derr == nil {
				if reason, uerr := abi.UnpackRevert(data); uerr == nil {
					return &RevertError{Reason: reason}
				}
			}
		}
	}
	return fmt.Errorf("failed to estimate gas: %w", err)
}

func isNonceError(err error) bool {
	if err == nil {
		return false
	}
	msg := strings.ToLower(err.Error())
	return strings.Contains(msg, "nonce too low") ||
		strings.Contains(msg, "nonce too high") ||
		strings.Contains(msg, "replacement transaction underpriced") ||
		strings.Contains(msg, "already known")
}

// estimateGasLimit estimates gas for the tx data and adds a safety buffer.
// The estimate runs in the pending block, the first one the transaction can
// go in. A transaction that reverts there is not sent: the error is then a
// *RevertError with the contract's reason.
// estimateRetries counts the gas estimates tried again after a Besu
// "Internal error": the experiments record it per run.
var estimateRetries atomic.Int64

// EstimateRetries is the number of gas estimates tried again so far.
func EstimateRetries() int64 { return estimateRetries.Load() }

func (bc *BlockchainClient) estimateGasLimit(ctx context.Context, data []byte) (uint64, error) {
	call := map[string]interface{}{
		"from":  bc.ethAddress,
		"to":    bc.contractAddress,
		"input": hexutil.Bytes(data),
	}
	var est hexutil.Uint64
	err := bc.client.Client().CallContext(ctx, &est, "eth_estimateGas", call, "pending")
	// Besu answers "Internal error" (-32603) when it simulates while a block
	// is being imported: the pending state is briefly unavailable. Tried
	// once more after the import; a revert has another code.
	var rpcErr rpc.Error
	if errors.As(err, &rpcErr) && rpcErr.ErrorCode() == -32603 {
		estimateRetries.Add(1)
		time.Sleep(100 * time.Millisecond)
		err = bc.client.Client().CallContext(ctx, &est, "eth_estimateGas", call, "pending")
	}
	if err != nil {
		return 0, revertReason(err)
	}

	// If simulation works, add a 30% buffer
	gas := uint64((uint64(est) * 130) / 100)

	// Standard safety floor
	if gas < 21_000 {
		gas = 21_000
	}
	return gas, nil
}

// sendSignedTransaction estimates, signs and sends a transaction. The nonce is
// taken and the transaction sent under one lock: on this zero-gas-price
// network, two transactions sent with the same nonce replace each other in
// the pool without an error (the prover and the juror of a sidecar send
// concurrently). On a nonce error the nonce is read again and the send
// retried once.
func (bc *BlockchainClient) sendSignedTransaction(ctx context.Context, data []byte) (string, error) {
	gasLimit, err := bc.estimateGasLimit(ctx, data)
	if err != nil {
		return "", err
	}

	bc.nonceLock.Lock()
	defer bc.nonceLock.Unlock()
	for attempt := 0; attempt < 2; attempt++ {
		tx := gethtypes.NewTransaction(bc.localNonce, bc.contractAddress, big.NewInt(0), gasLimit, bc.gasPrice, data)
		signedTx, err := gethtypes.SignTx(tx, gethtypes.NewEIP155Signer(bc.chainID), bc.privateKey)
		if err != nil {
			return "", fmt.Errorf("failed to sign tx: %v", err)
		}
		err = bc.client.SendTransaction(ctx, signedTx)
		if err == nil {
			bc.localNonce++
			return signedTx.Hash().Hex(), nil
		}
		if attempt == 0 && isNonceError(err) {
			if rerr := bc.refreshNonce(ctx); rerr != nil {
				return "", fmt.Errorf("failed to send tx: %v (nonce refresh: %v)", err, rerr)
			}
			continue
		}
		return "", fmt.Errorf("failed to send tx: %v", err)
	}
	return "", fmt.Errorf("failed to send tx after nonce refresh")
}

// WaitForReceipt waits until a transaction is mined. It fails with
// ErrReceiptTimeout if it is not mined in time, and with another error if it
// reverted.
func (bc *BlockchainClient) WaitForReceipt(txHashStr string, timeoutSec int) error {
	txHash := common.HexToHash(txHashStr)

	deadline := time.Now().Add(time.Duration(timeoutSec) * time.Second)
	logger.Debug("Waiting for transaction %s...", txHashStr)

	// Checked at once, then at each new block: the receipt is seen as soon as
	// the node imports the block that includes the transaction.
	for {
		next := bc.NextHead()
		receipt, err := bc.client.TransactionReceipt(context.Background(), txHash)
		if err == nil && receipt != nil {
			if receipt.Status == gethtypes.ReceiptStatusSuccessful {
				logger.Debug("Transaction %s mined in block %d", txHashStr, receipt.BlockNumber)
				return nil
			}
			return fmt.Errorf("transaction %s failed (status=0)", txHashStr)
		}
		// err: not mined yet (clients report it differently)
		if time.Now().After(deadline) {
			return fmt.Errorf("%w: tx %s", ErrReceiptTimeout, txHashStr)
		}
		WaitHead(next, nil)
	}
}

// -------------------------------------------------------------------------
// Contract Writes (Using Manual Packing + Send)
// -------------------------------------------------------------------------

func (bc *BlockchainClient) RegisterAgent(name string, wait bool, timeout int) (string, error) {
	data, err := bc.contractABI.Pack("RegisterAgent", name)
	if err != nil {
		return "", err
	}
	txHash, err := bc.sendSignedTransaction(context.Background(), data)
	if err != nil {
		return "", err
	}
	if wait {
		if err := bc.WaitForReceipt(txHash, timeout); err != nil {
			return txHash, err
		}
	}
	return txHash, nil
}

func (bc *BlockchainClient) RemoveAgent(wait bool, timeout int) (string, error) {
	data, err := bc.contractABI.Pack("RemoveAgent")
	if err != nil {
		return "", err
	}
	txHash, err := bc.sendSignedTransaction(context.Background(), data)
	if err != nil {
		return "", err
	}
	if wait {
		if err := bc.WaitForReceipt(txHash, timeout); err != nil {
			return txHash, err
		}
	}
	return txHash, nil
}

func (bc *BlockchainClient) ResetAttestationChain(wait bool, timeout int) (string, error) {
	data, err := bc.contractABI.Pack("ResetChain")
	if err != nil {
		return "", err
	}
	txHash, err := bc.sendSignedTransaction(context.Background(), data)
	if err != nil {
		return "", err
	}
	if wait {
		if err := bc.WaitForReceipt(txHash, timeout); err != nil {
			return txHash, err
		}
	}
	return txHash, nil
}

// Prove submits the rolling-hash digest plus its IterQ depth K
// (iterCount). The contract enforces 1 <= K <= MAX_ITER_COUNT (10000).
// For K=1 the digest is the single-shot measurement (no chaining); for
// K>1 it is the K-fold chain H_K = SHA256(H_{K-1} || m_K). boundBlock is
// the block the measurements are bound to, sent only to a contract that
// checks freshness (V3).
func (bc *BlockchainClient) Prove(attID string, freshSig string, iterCount uint32, boundBlock uint64, wait bool, timeout int) (string, error) {
	attIDBytes, err := TextToBytes32(attID)
	if err != nil {
		return "", err
	}
	sigBytes, err := AsBytes32(freshSig)
	if err != nil {
		return "", err
	}

	args := []interface{}{attIDBytes, sigBytes, iterCount}
	if bc.HasFreshness() {
		args = append(args, boundBlock)
	}
	data, err := bc.contractABI.Pack("Prove", args...)
	if err != nil {
		return "", err
	}

	txHash, err := bc.sendSignedTransaction(context.Background(), data)
	if err != nil {
		return "", err
	}
	if wait {
		if err := bc.WaitForReceipt(txHash, timeout); err != nil {
			return txHash, err
		}
	}
	return txHash, nil
}

func (bc *BlockchainClient) SubmitReference(attID string, refSig string, wait bool, timeout int) (string, error) {
	attIDBytes, err := TextToBytes32(attID)
	if err != nil {
		return "", err
	}
	sigBytes, err := AsBytes32(refSig)
	if err != nil {
		return "", err
	}

	data, err := bc.contractABI.Pack("SubmitReference", attIDBytes, sigBytes)
	if err != nil {
		return "", err
	}

	txHash, err := bc.sendSignedTransaction(context.Background(), data)
	if err != nil {
		return "", err
	}
	if wait {
		if err := bc.WaitForReceipt(txHash, timeout); err != nil {
			return txHash, err
		}
	}
	return txHash, nil
}

func (bc *BlockchainClient) Verify(attID string, verified bool, wait bool, timeout int) (string, error) {
	attIDBytes, err := TextToBytes32(attID)
	if err != nil {
		return "", err
	}
	data, err := bc.contractABI.Pack("Verify", attIDBytes, verified)
	if err != nil {
		return "", err
	}

	txHash, err := bc.sendSignedTransaction(context.Background(), data)
	if err != nil {
		return "", err
	}
	if wait {
		if err := bc.WaitForReceipt(txHash, timeout); err != nil {
			return txHash, err
		}
	}
	return txHash, nil
}

// Vote gives this agent's verdict as a juror of a V3 cycle.
// CheckVote rehearses Vote(attID) in the pending block without sending it:
// nil when this agent may vote now, a *RevertError otherwise (e.g. "caller is
// not in the jury"). The jury does not depend on the verdict.
func (bc *BlockchainClient) CheckVote(attID string) error {
	attIDBytes, err := TextToBytes32(attID)
	if err != nil {
		return err
	}
	data, err := bc.contractABI.Pack("Vote", attIDBytes, true)
	if err != nil {
		return err
	}
	_, err = bc.estimateGasLimit(context.Background(), data)
	return err
}

func (bc *BlockchainClient) Vote(attID string, verdict bool, wait bool, timeout int) (string, error) {
	attIDBytes, err := TextToBytes32(attID)
	if err != nil {
		return "", err
	}
	data, err := bc.contractABI.Pack("Vote", attIDBytes, verdict)
	if err != nil {
		return "", err
	}
	txHash, err := bc.sendSignedTransaction(context.Background(), data)
	if err != nil {
		return "", err
	}
	if wait {
		if err := bc.WaitForReceipt(txHash, timeout); err != nil {
			return txHash, err
		}
	}
	return txHash, nil
}

// Expire hands a V3 jury cycle to the SECaaS once its timeout has passed.
func (bc *BlockchainClient) Expire(attID string, wait bool, timeout int) (string, error) {
	attIDBytes, err := TextToBytes32(attID)
	if err != nil {
		return "", err
	}
	data, err := bc.contractABI.Pack("Expire", attIDBytes)
	if err != nil {
		return "", err
	}
	txHash, err := bc.sendSignedTransaction(context.Background(), data)
	if err != nil {
		return "", err
	}
	if wait {
		if err := bc.WaitForReceipt(txHash, timeout); err != nil {
			return txHash, err
		}
	}
	return txHash, nil
}

// -------------------------------------------------------------------------
// Contract Reads (Using rawEthCall)
// -------------------------------------------------------------------------

func (bc *BlockchainClient) IsRegistered() (bool, error) {
	data, err := bc.contractABI.Pack("IsRegistered", bc.ethAddress)
	if err != nil {
		return false, err
	}
	res, err := bc.rawEthCall(data)
	if err != nil {
		return false, err
	}

	// Unpack returns []interface{}; safest is Unpack then assert
	values, err := bc.contractABI.Unpack("IsRegistered", res)
	if err != nil {
		return false, err
	}
	if len(values) != 1 {
		return false, fmt.Errorf("IsRegistered: expected 1 return, got %d", len(values))
	}
	v, ok := values[0].(bool)
	if !ok {
		return false, fmt.Errorf("IsRegistered: unexpected type %T", values[0])
	}
	return v, nil
}

func (bc *BlockchainClient) GetAgentInfo(addr common.Address) (string, bool, error) {
	data, err := bc.contractABI.Pack("GetAgentInfo", addr, bc.ethAddress)
	if err != nil {
		return "", false, err
	}
	res, err := bc.rawEthCall(data)
	if err != nil {
		return "", false, err
	}
	values, err := bc.contractABI.Unpack("GetAgentInfo", res)
	if err != nil {
		return "", false, err
	}
	if len(values) != 2 {
		return "", false, fmt.Errorf("GetAgentInfo: expected 2 returns, got %d", len(values))
	}
	name, ok := values[0].(string)
	if !ok {
		return "", false, fmt.Errorf("GetAgentInfo: name type %T", values[0])
	}
	isReg, ok := values[1].(bool)
	if !ok {
		return "", false, fmt.Errorf("GetAgentInfo: registered type %T", values[1])
	}
	return name, isReg, nil
}

func (bc *BlockchainClient) IsVerifier(attID string) (bool, error) {
	attIDBytes, err := TextToBytes32(attID)
	if err != nil {
		return false, err
	}

	state, err := bc.GetAttestationState(attID)
	if err != nil || state == Closed {
		return false, nil
	}

	data, err := bc.contractABI.Pack("IsVerifier", attIDBytes, bc.ethAddress)
	if err != nil {
		return false, err
	}
	res, err := bc.rawEthCall(data)
	if err != nil {
		return false, nil
	}

	values, err := bc.contractABI.Unpack("IsVerifier", res)
	if err != nil {
		return false, nil
	}
	if len(values) != 1 {
		return false, nil
	}
	v, ok := values[0].(bool)
	if !ok {
		return false, nil
	}
	return v, nil
}

// HasJury tells whether the deployed contract draws a jury per cycle (V3).
func (bc *BlockchainClient) HasJury() bool {
	_, ok := bc.contractABI.Methods["Vote"]
	return ok
}

// HasFreshness tells whether the deployed contract takes the block the
// evidence is bound to (V3).
func (bc *BlockchainClient) HasFreshness() bool {
	return len(bc.contractABI.Methods["Prove"].Inputs) == 4
}

// ParentBlock returns the number and hash of the block before the latest one,
// as the node reports them (the hash the contract's blockhash returns).
// Evidence is bound to it rather than to the latest block: the node simulates
// Prove (gas estimate) in the latest block, where that block's hash is not
// readable yet.
func (bc *BlockchainClient) ParentBlock() (uint64, [32]byte, error) {
	var head struct {
		Number     hexutil.Uint64 `json:"number"`
		ParentHash common.Hash    `json:"parentHash"`
	}
	err := bc.client.Client().CallContext(context.Background(), &head, "eth_getBlockByNumber", "latest", false)
	if err == nil && head.Number == 0 {
		err = errors.New("the chain has no block before the genesis")
	}
	return uint64(head.Number) - 1, head.ParentHash, err
}

func (bc *BlockchainClient) Address() common.Address {
	return bc.ethAddress
}

func (bc *BlockchainClient) BlockNumber() (uint64, error) {
	return bc.client.BlockNumber(context.Background())
}

// TxBlockNumber returns the block a transaction was mined in.
func (bc *BlockchainClient) TxBlockNumber(txHashStr string) (uint64, error) {
	receipt, err := bc.client.TransactionReceipt(context.Background(), common.HexToHash(txHashStr))
	if err != nil {
		return 0, err
	}
	return receipt.BlockNumber.Uint64(), nil
}

// JuryTimeoutBlocks returns the number of blocks after its evidence block
// from which a V3 jury cycle can be expired.
func (bc *BlockchainClient) JuryTimeoutBlocks() (uint64, error) {
	return bc.fixedUint("juryTimeoutBlocks", &bc.juryTimeout)
}

// FreshnessBlocks returns the age, in blocks, past which V3 rejects evidence.
func (bc *BlockchainClient) FreshnessBlocks() (uint64, error) {
	return bc.fixedUint("freshnessBlocks", &bc.freshness)
}

// fixedUint reads a uint setting fixed at deployment once, then serves it
// from cache.
func (bc *BlockchainClient) fixedUint(getter string, cache *atomic.Uint64) (uint64, error) {
	if v := cache.Load(); v != 0 {
		return v, nil
	}
	data, err := bc.contractABI.Pack(getter)
	if err != nil {
		return 0, err
	}
	res, err := bc.rawEthCall(data)
	if err != nil {
		return 0, err
	}
	values, err := bc.contractABI.Unpack(getter, res)
	if err != nil {
		return 0, err
	}
	v, ok := values[0].(*big.Int)
	if !ok {
		return 0, fmt.Errorf("%s: unexpected type %T", getter, values[0])
	}
	cache.Store(v.Uint64())
	return v.Uint64(), nil
}

// JuryVotes returns the frozen jury of a V3 cycle and what each juror
// voted, e.g. "robot2=valid, robot3=invalid, robot4=no vote". Both are
// empty before the first vote.
func (bc *BlockchainClient) JuryVotes(attID string) ([]common.Address, string, error) {
	attIDBytes, err := TextToBytes32(attID)
	if err != nil {
		return nil, "", err
	}
	data, err := bc.contractABI.Pack("GetJury", attIDBytes)
	if err != nil {
		return nil, "", err
	}
	res, err := bc.rawEthCall(data)
	if err != nil {
		return nil, "", err
	}
	values, err := bc.contractABI.Unpack("GetJury", res)
	if err != nil {
		return nil, "", err
	}
	jurors, ok1 := values[0].([]common.Address)
	votes, ok2 := values[1].([]uint8)
	if len(values) != 2 || !ok1 || !ok2 || len(jurors) != len(votes) {
		return nil, "", fmt.Errorf("GetJury: unexpected return %v", values)
	}
	parts := make([]string, len(jurors))
	for i, j := range jurors {
		name, err := bc.agentName(j)
		if err != nil {
			return nil, "", err
		}
		parts[i] = name + "=" + [...]string{"no vote", "invalid", "valid"}[votes[i]]
	}
	return jurors, strings.Join(parts, ", "), nil
}

// agentName returns the registered name of an agent, read once.
func (bc *BlockchainClient) agentName(addr common.Address) (string, error) {
	if name, ok := bc.agentNames.Load(addr); ok {
		return name.(string), nil
	}
	name, _, err := bc.GetAgentInfo(addr)
	if err != nil {
		return "", err
	}
	bc.agentNames.Store(addr, name)
	return name, nil
}

func (bc *BlockchainClient) GetAttestationState(attID string) (AttestationState, error) {
	attIDBytes, err := TextToBytes32(attID)
	if err != nil {
		return 0, err
	}
	data, err := bc.contractABI.Pack("GetAttestationState", attIDBytes)
	if err != nil {
		return 0, err
	}
	res, err := bc.rawEthCall(data)
	if err != nil {
		return 0, err
	}

	values, err := bc.contractABI.Unpack("GetAttestationState", res)
	if err != nil {
		return 0, err
	}
	if len(values) != 1 {
		return 0, fmt.Errorf("GetAttestationState: expected 1 return, got %d", len(values))
	}
	u8, ok := values[0].(uint8)
	if !ok {
		// sometimes enums unpack as *big.Int
		if bi, ok2 := values[0].(*big.Int); ok2 {
			u8 = uint8(bi.Uint64())
		} else {
			return 0, fmt.Errorf("GetAttestationState: unexpected type %T", values[0])
		}
	}
	return AttestationState(u8), nil
}

func (bc *BlockchainClient) GetAttestationInfo(attID string) (common.Address, common.Address, uint8, error) {
	attIDBytes, err := TextToBytes32(attID)
	if err != nil {
		return common.Address{}, common.Address{}, 0, err
	}
	data, err := bc.contractABI.Pack("GetAttestationInfo", attIDBytes)
	if err != nil {
		return common.Address{}, common.Address{}, 0, err
	}
	res, err := bc.rawEthCall(data)
	if err != nil {
		return common.Address{}, common.Address{}, 0, err
	}
	values, err := bc.contractABI.Unpack("GetAttestationInfo", res)
	if err != nil {
		return common.Address{}, common.Address{}, 0, err
	}
	if len(values) != 3 {
		return common.Address{}, common.Address{}, 0, fmt.Errorf("GetAttestationInfo: expected 3 returns, got %d", len(values))
	}

	prover, ok := values[0].(common.Address)
	if !ok {
		return common.Address{}, common.Address{}, 0, fmt.Errorf("GetAttestationInfo: prover type %T", values[0])
	}
	verifier, ok := values[1].(common.Address)
	if !ok {
		return common.Address{}, common.Address{}, 0, fmt.Errorf("GetAttestationInfo: verifier type %T", values[1])
	}
	var result uint8
	switch v := values[2].(type) {
	case uint8:
		result = v
	case *big.Int:
		result = uint8(v.Uint64())
	default:
		return common.Address{}, common.Address{}, 0, fmt.Errorf("GetAttestationInfo: result type %T", values[2])
	}
	return prover, verifier, result, nil
}

func (bc *BlockchainClient) GetAttestationSignatures(attID string) (string, string, uint32, uint64, error) {
	attIDBytes, err := TextToBytes32(attID)
	if err != nil {
		return "", "", 0, 0, err
	}
	data, err := bc.contractABI.Pack("GetAttestationSignatures", attIDBytes, bc.ethAddress)
	if err != nil {
		return "", "", 0, 0, err
	}
	res, err := bc.rawEthCall(data)
	if err != nil {
		return "", "", 0, 0, err
	}

	values, err := bc.contractABI.Unpack("GetAttestationSignatures", res)
	if err != nil {
		return "", "", 0, 0, err
	}
	if len(values) < 3 {
		return "", "", 0, 0, fmt.Errorf("GetAttestationSignatures: expected 3 or 4 returns, got %d", len(values))
	}

	fresh, ok := values[0].([32]byte)
	if !ok {
		return "", "", 0, 0, fmt.Errorf("GetAttestationSignatures: fresh type %T", values[0])
	}
	ref, ok := values[1].([32]byte)
	if !ok {
		return "", "", 0, 0, fmt.Errorf("GetAttestationSignatures: ref type %T", values[1])
	}
	iterCount, ok := values[2].(uint32)
	if !ok {
		return "", "", 0, 0, fmt.Errorf("GetAttestationSignatures: iterCount type %T", values[2])
	}
	var boundBlock uint64
	if len(values) > 3 {
		if boundBlock, ok = values[3].(uint64); !ok {
			return "", "", 0, 0, fmt.Errorf("GetAttestationSignatures: boundBlock type %T", values[3])
		}
	}

	return "0x" + hex.EncodeToString(fresh[:]), "0x" + hex.EncodeToString(ref[:]), iterCount, boundBlock, nil
}

// BlockHash returns the hash of block n as the node reports it.
func (bc *BlockchainClient) BlockHash(n uint64) ([32]byte, error) {
	var block struct {
		Hash common.Hash `json:"hash"`
	}
	err := bc.client.Client().CallContext(context.Background(), &block, "eth_getBlockByNumber", hexutil.EncodeUint64(n), false)
	if err == nil && block.Hash == (common.Hash{}) {
		err = fmt.Errorf("block %d not found", n)
	}
	return block.Hash, err
}

// -------------------------------------------------------------------------
// Misc
// -------------------------------------------------------------------------

type TxReceiptInfo struct {
	TxHash            string   `json:"tx_hash"`
	Status            uint64   `json:"status"`
	GasUsed           uint64   `json:"gas_used"`
	EffectiveGasPrice *big.Int `json:"effective_gas_price"`
	BlockNumber       uint64   `json:"block_number"`
	BlockTimestamp    uint64   `json:"block_timestamp"`
	BlockSize         uint64   `json:"block_size"`
	TransactionCount  uint64   `json:"transaction_count"`
}

func (bc *BlockchainClient) GetTransactionReceipt(txHashStr string) (*TxReceiptInfo, error) {
	txHash := common.HexToHash(txHashStr)

	receipt, err := bc.client.TransactionReceipt(context.Background(), txHash)
	if err != nil {
		return nil, err
	}
	if receipt == nil {
		return nil, fmt.Errorf("receipt not found for tx %s", txHashStr)
	}

	header, err := bc.client.HeaderByNumber(context.Background(), receipt.BlockNumber)
	if err != nil {
		return nil, err
	}

	block, err := bc.client.BlockByNumber(context.Background(), receipt.BlockNumber)
	if err != nil {
		return nil, err
	}

	return &TxReceiptInfo{
		TxHash:            txHashStr,
		Status:            receipt.Status,
		GasUsed:           receipt.GasUsed,
		EffectiveGasPrice: receipt.EffectiveGasPrice,
		BlockNumber:       receipt.BlockNumber.Uint64(),
		BlockTimestamp:    header.Time,
		BlockSize:         uint64(block.Size()),
		TransactionCount:  uint64(len(block.Transactions())),
	}, nil
}

func (bc *BlockchainClient) Close() {
	if bc.client != nil {
		bc.client.Close()
	}
}
