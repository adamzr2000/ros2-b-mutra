package blockchain

import (
	"context"
	"sync"
	"time"

	"app/internal/logger"

	gethtypes "github.com/ethereum/go-ethereum/core/types"
	"github.com/ethereum/go-ethereum/ethclient"
)

// headWaitMax bounds a wait for the next block: a check runs at least this
// often, whatever the notifications.
const headWaitMax = 5 * time.Second

// heads tells the waiters when the node imports a block. It follows the
// node's newHeads subscription over WebSocket; without a WebSocket URL, or
// while the subscription is down, it ticks every fallback interval instead.
type heads struct {
	mu   sync.Mutex
	next chan struct{} // closed at the next block
	num  uint64        // latest block notified, 0 when polling
	at   time.Time     // when it arrived
	live bool          // following the subscription
}

func newHeads() *heads {
	return &heads{next: make(chan struct{})}
}

func (h *heads) fire(num uint64, live bool) {
	h.mu.Lock()
	h.num, h.at, h.live = num, time.Now(), live
	close(h.next)
	h.next = make(chan struct{})
	h.mu.Unlock()
}

func (h *heads) run(wsURL string, fallback time.Duration) {
	for {
		if wsURL != "" {
			h.follow(wsURL)
			logger.Warn("Block notifications lost: polling every %v, reconnecting in 5s", fallback)
		}
		retry := time.Now().Add(5 * time.Second)
		for wsURL == "" || time.Now().Before(retry) {
			time.Sleep(fallback)
			h.fire(0, false)
		}
	}
}

// follow returns when the subscription cannot start or ends.
func (h *heads) follow(wsURL string) {
	client, err := ethclient.Dial(wsURL)
	if err != nil {
		logger.Error("WebSocket %s: %v", wsURL, err)
		return
	}
	defer client.Close()
	headers := make(chan *gethtypes.Header, 16)
	sub, err := client.SubscribeNewHead(context.Background(), headers)
	if err != nil {
		logger.Error("newHeads subscription on %s: %v", wsURL, err)
		return
	}
	defer sub.Unsubscribe()
	logger.Info("Following new blocks on %s", wsURL)
	for {
		select {
		case header := <-headers:
			h.fire(header.Number.Uint64(), true)
		case err := <-sub.Err():
			logger.Error("newHeads subscription on %s ended: %v", wsURL, err)
			return
		}
	}
}

// FollowHeads starts the block notifications. Call once, before any wait.
func (bc *BlockchainClient) FollowHeads(wsURL string, fallback time.Duration) {
	go bc.heads.run(wsURL, fallback)
}

// NextHead is closed when the node imports the next block. Take it before
// checking the chain, then wait on it: a block imported in between is not
// missed.
func (bc *BlockchainClient) NextHead() <-chan struct{} {
	bc.heads.mu.Lock()
	defer bc.heads.mu.Unlock()
	return bc.heads.next
}

// WaitHead waits for the channel of NextHead, at most headWaitMax. False when
// stop is closed first.
func WaitHead(next <-chan struct{}, stop <-chan struct{}) bool {
	timer := time.NewTimer(headWaitMax)
	defer timer.Stop()
	select {
	case <-next:
	case <-timer.C:
	case <-stop:
		return false
	}
	return true
}

// LastHead is the latest block notified and when it arrived; ok is false
// while polling.
func (bc *BlockchainClient) LastHead() (num uint64, at time.Time, ok bool) {
	bc.heads.mu.Lock()
	defer bc.heads.mu.Unlock()
	return bc.heads.num, bc.heads.at, bc.heads.live
}

// HeadsMode is "websocket" while following the subscription, else "polling".
func (bc *BlockchainClient) HeadsMode() string {
	if _, _, live := bc.LastHead(); live {
		return "websocket"
	}
	return "polling"
}
