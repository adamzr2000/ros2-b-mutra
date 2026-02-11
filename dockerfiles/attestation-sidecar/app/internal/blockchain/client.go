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
	"time"

	"app/internal/logger"
	// types.go is in the same package 'blockchain'

	"github.com/ethereum/go-ethereum/accounts/abi"
	"github.com/ethereum/go-ethereum/common"
	gethtypes "github.com/ethereum/go-ethereum/core/types"
	"github.com/ethereum/go-ethereum/crypto"
	"github.com/ethereum/go-ethereum/ethclient"
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
	ethNodeURL      string

	nonceLock  sync.Mutex
	localNonce uint64
}

// Helper struct for loading ABI JSON (Hardhat-style artifact: {"abi":[...]} )
type ABIFile struct {
	ABI json.RawMessage `json:"abi"`
}

// NewBlockchainClient initializes the connection and contract binding.
// Updated to remove redundant abiPath arg to match main.go
func NewBlockchainClient(ethAddressStr, privateKeyHex, ethNodeURL, contractAddrStr string, contractABI interface{}) (*BlockchainClient, error) {
	// 1. Connect
	client, err := ethclient.Dial(ethNodeURL)
	if err != nil {
		return nil, fmt.Errorf("failed to connect to eth node: %v", err)
	}

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

	bc := &BlockchainClient{
		client:          client,
		privateKey:      privateKey,
		ethAddress:      derivedAddr,
		contractAddress: common.HexToAddress(contractAddrStr),
		contractABI:     parsedABI,
		chainID:         chainID,
		ethNodeURL:      ethNodeURL,
		localNonce:      nonce,
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
}

type rpcResponse struct {
	Result string    `json:"result"`
	Error  *rpcError `json:"error"`
}

// rawEthCall executes eth_call via direct JSON-RPC.
// If from is nil, we omit it (often the most compatible behavior).
func (bc *BlockchainClient) rawEthCall(from *common.Address, data []byte) ([]byte, error) {
	type callParams struct {
		From string `json:"from,omitempty"`
		To   string `json:"to"`
		Data string `json:"data"`
	}

	params := callParams{
		To:   bc.contractAddress.Hex(),
		Data: "0x" + hex.EncodeToString(data),
	}
	if from != nil {
		params.From = from.Hex()
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

// sendSignedTransaction constructs, signs, and sends a transaction manually.
// - Increments nonce only after successful SendTransaction.
// - Retries once after refreshing nonce on nonce-related errors.
func (bc *BlockchainClient) sendSignedTransaction(ctx context.Context, data []byte, gasLimit uint64) (string, error) {
	for attempt := 0; attempt < 2; attempt++ {
		// read current nonce without changing it yet
		bc.nonceLock.Lock()
		nonce := bc.localNonce
		bc.nonceLock.Unlock()

		gasPrice, err := bc.client.SuggestGasPrice(ctx)
		if err != nil {
			return "", fmt.Errorf("failed to get gas price: %v", err)
		}
		// bump gas price by 25%
		bump := new(big.Int).Mul(gasPrice, big.NewInt(125))
		bump.Div(bump, big.NewInt(100))

		tx := gethtypes.NewTransaction(
			nonce,
			bc.contractAddress,
			big.NewInt(0),
			gasLimit,
			bump,
			data,
		)

		signedTx, err := gethtypes.SignTx(tx, gethtypes.NewEIP155Signer(bc.chainID), bc.privateKey)
		if err != nil {
			return "", fmt.Errorf("failed to sign tx: %v", err)
		}

		err = bc.client.SendTransaction(ctx, signedTx)
		if err == nil {
			// increment nonce only on success
			bc.nonceLock.Lock()
			// keep monotonic
			if bc.localNonce == nonce {
				bc.localNonce++
			} else if bc.localNonce < nonce+1 {
				bc.localNonce = nonce + 1
			}
			bc.nonceLock.Unlock()
			return signedTx.Hash().Hex(), nil
		}

		if attempt == 0 && isNonceError(err) {
			bc.nonceLock.Lock()
			_ = bc.refreshNonce(ctx)
			bc.nonceLock.Unlock()
			continue
		}

		return "", fmt.Errorf("failed to send tx: %v", err)
	}
	return "", fmt.Errorf("failed to send tx after nonce refresh")
}

func (bc *BlockchainClient) waitForReceipt(txHashStr string, timeoutSec int) error {
	txHash := common.HexToHash(txHashStr)

	ctx, cancel := context.WithTimeout(context.Background(), time.Duration(timeoutSec)*time.Second)
	defer cancel()

	ticker := time.NewTicker(500 * time.Millisecond)
	defer ticker.Stop()

	logger.Debug("Waiting for transaction %s...", txHashStr)

	for {
		select {
		case <-ctx.Done():
			return fmt.Errorf("timeout waiting for tx %s", txHashStr)
		case <-ticker.C:
			receipt, err := bc.client.TransactionReceipt(context.Background(), txHash)
			if err != nil {
				// common case: not mined yet (different clients use different errors)
				continue
			}
			if receipt == nil {
				continue
			}
			if receipt.Status == gethtypes.ReceiptStatusSuccessful {
				logger.Debug("Transaction %s mined in block %d", txHashStr, receipt.BlockNumber)
				return nil
			}
			return fmt.Errorf("transaction %s failed (status=0)", txHashStr)
		}
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
	txHash, err := bc.sendSignedTransaction(context.Background(), data, 300000)
	if err != nil {
		return "", err
	}
	if wait {
		if err := bc.waitForReceipt(txHash, timeout); err != nil {
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
	txHash, err := bc.sendSignedTransaction(context.Background(), data, 200000)
	if err != nil {
		return "", err
	}
	if wait {
		if err := bc.waitForReceipt(txHash, timeout); err != nil {
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
	txHash, err := bc.sendSignedTransaction(context.Background(), data, 200000)
	if err != nil {
		return "", err
	}
	if wait {
		if err := bc.waitForReceipt(txHash, timeout); err != nil {
			return txHash, err
		}
	}
	return txHash, nil
}

func (bc *BlockchainClient) SendEvidence(attID string, freshSigs []string, wait bool, timeout int) (string, error) {
	attIDBytes, err := TextToBytes32(attID)
	if err != nil {
		return "", err
	}
	if len(freshSigs) != 3 {
		return "", errors.New("expected 3 signatures")
	}

	var sigTriplet [3][32]byte
	for i, s := range freshSigs {
		b, err := AsBytes32(s)
		if err != nil {
			return "", err
		}
		sigTriplet[i] = b
	}

	data, err := bc.contractABI.Pack("SendEvidence", attIDBytes, sigTriplet)
	if err != nil {
		return "", err
	}

	txHash, err := bc.sendSignedTransaction(context.Background(), data, 400000)
	if err != nil {
		return "", err
	}
	if wait {
		if err := bc.waitForReceipt(txHash, timeout); err != nil {
			return txHash, err
		}
	}
	return txHash, nil
}

func (bc *BlockchainClient) SendRefSignatures(attID string, refSigs []string, wait bool, timeout int) (string, error) {
	attIDBytes, err := TextToBytes32(attID)
	if err != nil {
		return "", err
	}
	if len(refSigs) != 3 {
		return "", errors.New("expected 3 signatures")
	}

	var sigTriplet [3][32]byte
	for i, s := range refSigs {
		b, err := AsBytes32(s)
		if err != nil {
			return "", err
		}
		sigTriplet[i] = b
	}

	data, err := bc.contractABI.Pack("SendRefSignatures", attIDBytes, sigTriplet)
	if err != nil {
		return "", err
	}

	txHash, err := bc.sendSignedTransaction(context.Background(), data, 400000)
	if err != nil {
		return "", err
	}
	if wait {
		if err := bc.waitForReceipt(txHash, timeout); err != nil {
			return txHash, err
		}
	}
	return txHash, nil
}

func (bc *BlockchainClient) SendAttestationResult(attID string, verified bool, wait bool, timeout int) (string, error) {
	attIDBytes, err := TextToBytes32(attID)
	if err != nil {
		return "", err
	}
	data, err := bc.contractABI.Pack("CloseAttestationProcess", attIDBytes, verified)
	if err != nil {
		return "", err
	}

	txHash, err := bc.sendSignedTransaction(context.Background(), data, 300000)
	if err != nil {
		return "", err
	}
	if wait {
		if err := bc.waitForReceipt(txHash, timeout); err != nil {
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
	res, err := bc.rawEthCall(nil, data)
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

func (bc *BlockchainClient) GetAgentInfo(addr common.Address) (string, bool, *big.Int, error) {
	data, err := bc.contractABI.Pack("GetAgentInfo", addr, bc.ethAddress)
	if err != nil {
		return "", false, nil, err
	}
	res, err := bc.rawEthCall(nil, data)
	if err != nil {
		return "", false, nil, err
	}

	values, err := bc.contractABI.Unpack("GetAgentInfo", res)
	if err != nil {
		return "", false, nil, err
	}
	if len(values) < 3 {
		return "", false, nil, fmt.Errorf("GetAgentInfo: expected 3 returns, got %d", len(values))
	}

	name, ok := values[0].(string)
	if !ok {
		return "", false, nil, fmt.Errorf("GetAgentInfo: name type %T", values[0])
	}
	isReg, ok := values[1].(bool)
	if !ok {
		return "", false, nil, fmt.Errorf("GetAgentInfo: registered type %T", values[1])
	}

	// 3rd return is bytes32[]; compute len robustly
	count := big.NewInt(0)
	switch v := values[2].(type) {
	case [][32]byte:
		count = big.NewInt(int64(len(v)))
	case []common.Hash:
		count = big.NewInt(int64(len(v)))
	case []interface{}:
		count = big.NewInt(int64(len(v)))
	default:
		// keep 0, but avoid panic
		count = big.NewInt(0)
	}

	return name, isReg, count, nil
}

func (bc *BlockchainClient) IsProver(attID string) (bool, error) {
	attIDBytes, err := TextToBytes32(attID)
	if err != nil {
		return false, err
	}

	// Check state first (avoid revert when Closed/nonexistent depending on contract)
	state, err := bc.GetAttestationState(attID)
	if err != nil || state == Closed {
		return false, nil
	}

	data, err := bc.contractABI.Pack("IsProver", attIDBytes, bc.ethAddress)
	if err != nil {
		return false, err
	}
	res, err := bc.rawEthCall(nil, data)
	if err != nil {
		return false, nil
	}

	values, err := bc.contractABI.Unpack("IsProver", res)
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
	res, err := bc.rawEthCall(nil, data)
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

func (bc *BlockchainClient) GetAttestationState(attID string) (AttestationState, error) {
	attIDBytes, err := TextToBytes32(attID)
	if err != nil {
		return 0, err
	}
	data, err := bc.contractABI.Pack("GetAttestationState", attIDBytes)
	if err != nil {
		return 0, err
	}
	res, err := bc.rawEthCall(nil, data)
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

func (bc *BlockchainClient) GetAttestationInfo(attID string) (common.Address, common.Address, uint8, *big.Int, error) {
	attIDBytes, err := TextToBytes32(attID)
	if err != nil {
		return common.Address{}, common.Address{}, 0, nil, err
	}
	data, err := bc.contractABI.Pack("GetAttestationInfo", attIDBytes)
	if err != nil {
		return common.Address{}, common.Address{}, 0, nil, err
	}
	res, err := bc.rawEthCall(nil, data)
	if err != nil {
		return common.Address{}, common.Address{}, 0, nil, err
	}
	values, err := bc.contractABI.Unpack("GetAttestationInfo", res)
	if err != nil {
		return common.Address{}, common.Address{}, 0, nil, err
	}
	if len(values) < 4 {
		return common.Address{}, common.Address{}, 0, nil, fmt.Errorf("GetAttestationInfo: expected 4 returns, got %d", len(values))
	}

	prover, ok := values[0].(common.Address)
	if !ok {
		return common.Address{}, common.Address{}, 0, nil, fmt.Errorf("GetAttestationInfo: prover type %T", values[0])
	}
	verifier, ok := values[1].(common.Address)
	if !ok {
		return common.Address{}, common.Address{}, 0, nil, fmt.Errorf("GetAttestationInfo: verifier type %T", values[1])
	}

	var result uint8
	switch v := values[2].(type) {
	case uint8:
		result = v
	case *big.Int:
		result = uint8(v.Uint64())
	default:
		return common.Address{}, common.Address{}, 0, nil, fmt.Errorf("GetAttestationInfo: result type %T", values[2])
	}

	ts, ok := values[3].(*big.Int)
	if !ok {
		return common.Address{}, common.Address{}, 0, nil, fmt.Errorf("GetAttestationInfo: timestamp type %T", values[3])
	}

	return prover, verifier, result, ts, nil
}

func (bc *BlockchainClient) GetProverAddress(attID string) (common.Address, error) {
	attIDBytes, err := TextToBytes32(attID)
	if err != nil {
		return common.Address{}, err
	}
	data, err := bc.contractABI.Pack("GetProverAddress", attIDBytes, bc.ethAddress)
	if err != nil {
		return common.Address{}, err
	}
	res, err := bc.rawEthCall(nil, data)
	if err != nil {
		return common.Address{}, err
	}

	values, err := bc.contractABI.Unpack("GetProverAddress", res)
	if err != nil {
		return common.Address{}, err
	}
	if len(values) != 1 {
		return common.Address{}, fmt.Errorf("GetProverAddress: expected 1 return, got %d", len(values))
	}
	addr, ok := values[0].(common.Address)
	if !ok {
		return common.Address{}, fmt.Errorf("GetProverAddress: unexpected type %T", values[0])
	}
	return addr, nil
}

func (bc *BlockchainClient) GetAttestationSignatures(attID string) ([]string, []string, error) {
	attIDBytes, err := TextToBytes32(attID)
	if err != nil {
		return nil, nil, err
	}
	data, err := bc.contractABI.Pack("GetAttestationSignatures", attIDBytes, bc.ethAddress)
	if err != nil {
		return nil, nil, err
	}
	res, err := bc.rawEthCall(nil, data)
	if err != nil {
		return nil, nil, err
	}

	values, err := bc.contractABI.Unpack("GetAttestationSignatures", res)
	if err != nil {
		return nil, nil, err
	}
	if len(values) < 2 {
		return nil, nil, fmt.Errorf("GetAttestationSignatures: expected 2 returns, got %d", len(values))
	}

	fresh, ok := values[0].([3][32]byte)
	if !ok {
		return nil, nil, fmt.Errorf("GetAttestationSignatures: fresh type %T", values[0])
	}
	ref, ok := values[1].([3][32]byte)
	if !ok {
		return nil, nil, fmt.Errorf("GetAttestationSignatures: ref type %T", values[1])
	}

	toHex := func(arr [3][32]byte) []string {
		out := make([]string, 3)
		for i, b := range arr {
			out[i] = "0x" + hex.EncodeToString(b[:])
		}
		return out
	}
	return toHex(fresh), toHex(ref), nil
}

func (bc *BlockchainClient) GetAttestationChain() ([][32]byte, error) {
	data, err := bc.contractABI.Pack("GetAttestationChain")
	if err != nil {
		return nil, err
	}
	res, err := bc.rawEthCall(nil, data)
	if err != nil {
		return nil, err
	}

	values, err := bc.contractABI.Unpack("GetAttestationChain", res)
	if err != nil {
		return nil, err
	}
	if len(values) != 1 {
		return nil, fmt.Errorf("GetAttestationChain: expected 1 return, got %d", len(values))
	}

	// Most clients decode bytes32[] into [][32]byte directly.
	switch v := values[0].(type) {
	case [][32]byte:
		return v, nil
	case []common.Hash:
		out := make([][32]byte, 0, len(v))
		for _, h := range v {
			var b [32]byte
			copy(b[:], h.Bytes())
			out = append(out, b)
		}
		return out, nil
	case []interface{}:
		out := make([][32]byte, 0, len(v))
		for _, it := range v {
			switch t := it.(type) {
			case [32]byte:
				out = append(out, t)
			case common.Hash:
				var b [32]byte
				copy(b[:], t.Bytes())
				out = append(out, b)
			default:
				return nil, fmt.Errorf("GetAttestationChain: unexpected element type %T", it)
			}
		}
		return out, nil
	default:
		return nil, fmt.Errorf("GetAttestationChain: unexpected type %T", values[0])
	}
}

// -------------------------------------------------------------------------
// Misc
// -------------------------------------------------------------------------

func (bc *BlockchainClient) PeriodicAttestationChainDisplay(chainDisplayN int, interval int) {
	ticker := time.NewTicker(time.Duration(interval) * time.Second)
	defer ticker.Stop()
	for {
		bc.safeDisplay(chainDisplayN)
		<-ticker.C
	}
}

func (bc *BlockchainClient) safeDisplay(n int) {
	chain, err := bc.GetAttestationChain()
	if err != nil {
		logger.Error("Failed to display chain: %v", err)
		return
	}
	tableStr := BuildAttestationChainTable(
		chain,
		bc.GetAttestationInfo,
		bc.GetAgentInfo,
		"0xed9d02e382b34818e88b88a309c7fe71e65f419d",
		n,
	)
	logger.Info("\n%s", tableStr)
}

func (bc *BlockchainClient) Close() {
	if bc.client != nil {
		bc.client.Close()
	}
}