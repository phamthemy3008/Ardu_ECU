// Command serial_web_bridge is a standalone (no install, no runtime deps)
// bridge for the ECU_TestV1_EGT_DRY_START_PATCH firmware.
//
// It serves the ECU control UI (page.html - the single UI; the firmware no
// longer serves its own web page) on a local HTTP port, and talks to the ECU
// over ONE of two connection modes:
//
//   wire : Serial/USB-TTL to GSU_DEBUG_UART1. The Terminal tab shows ALL raw
//          Serial output (boot/brownout logs, command replies).
//   wifi : the ECU's SoftAP - proxies /api and /cmd over HTTP to the ECU. The
//          Terminal tab then only shows the ECU Event Log (WiFi doesn't survive
//          a reset, so it can't capture reset logs).
//
// Build (Windows target, from any OS with Go installed):
//   GOOS=windows GOARCH=amd64 go build -o serial_web_bridge.exe .
//
// Run:
//   serial_web_bridge.exe                     (asks mode/target interactively)
//   serial_web_bridge.exe wire COM5
//   serial_web_bridge.exe wifi 192.168.4.1
// optional: [baud] [http-port] as extra args, e.g. wire COM5 115200 8080
package main

import (
	"bufio"
	_ "embed"
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"net/url"
	"os"
	"strconv"
	"strings"
	"sync"
	"time"
)

//go:embed page.html
var pageHTML []byte

const (
	termMax       = 2000
	statusPollWire = 400 * time.Millisecond
	statusPollWiFi = 500 * time.Millisecond
	wifiTimeout    = 1500 * time.Millisecond
)

var (
	mu         sync.Mutex
	lastStatus = `{"error":"chua co du lieu tu ECU"}`
	termLines  []string
	termBase   int
	connState  = "-"

	mode     string // "wire" | "wifi"
	wifiBase string

	portMu sync.Mutex // guards port across the reader/poller goroutines and Reconnect
	port   *comPort

	lastRxMs   int64  // unix ms of last successfully parsed serial line, 0 = none yet (wire mode)
	wireInfo   string // e.g. "COM5 @ 115200", for display in connState
	wireTarget string // saved COM port name, for Reconnect
	wireBaud   int    // saved baud, for Reconnect
)

func curPort() *comPort {
	portMu.Lock()
	defer portMu.Unlock()
	return port
}

const wireStaleAfter = 2 * time.Second // no serial line for this long -> report as disconnected

func addTerm(line string) {
	mu.Lock()
	defer mu.Unlock()
	termLines = append(termLines, line)
	if len(termLines) > termMax {
		drop := len(termLines) - termMax
		termLines = termLines[drop:]
		termBase += drop
	}
}

func isStatusJSON(line string) bool {
	return strings.HasPrefix(line, "{") && strings.Contains(line, `"mode"`)
}

// ---------------- wire mode ----------------

func wireReader() {
	buf := make([]byte, 512)
	var acc []byte
	for {
		p := curPort()
		if p == nil {
			acc = acc[:0]
			time.Sleep(200 * time.Millisecond)
			continue
		}
		n := p.Read(buf)
		if n <= 0 {
			continue
		}
		acc = append(acc, buf[:n]...)
		for {
			i := indexByte(acc, '\n')
			if i < 0 {
				break
			}
			line := strings.TrimRight(string(acc[:i]), "\r")
			acc = acc[i+1:]
			if line == "" {
				continue
			}
			if isStatusJSON(line) {
				mu.Lock()
				lastStatus = line
				// "OK" in the UI must mean a real status round-trip succeeded, not just
				// that some byte arrived (boot banner, "Unknown command", etc.) - only a
				// parsed apijson reply advances lastRxMs.
				lastRxMs = time.Now().UnixMilli()
				mu.Unlock()
			} else {
				addTerm(line)
			}
		}
	}
}

func wirePoller() {
	for {
		if p := curPort(); p != nil {
			p.WriteLine("apijson")
		}
		time.Sleep(statusPollWire)
	}
}

// reconnectWire closes and re-opens the saved COM port. Used by the /reconnect
// endpoint so the user can recover after unplug/replug or an ECU brownout
// without restarting the whole bridge.
func reconnectWire() {
	portMu.Lock()
	if port != nil {
		port.Close()
		port = nil
	}
	np, err := openComPort(wireTarget, uint32(wireBaud))
	if err == nil {
		port = np
	}
	portMu.Unlock()
	mu.Lock()
	lastRxMs = 0
	if err != nil {
		connState = "LOI MO LAI CONG: " + err.Error() + " (" + wireInfo + ")"
	}
	mu.Unlock()
	if err != nil {
		addTerm("[bridge] ket noi lai that bai: " + err.Error())
	} else {
		addTerm("[bridge] da mo lai cong " + wireInfo)
	}
}

// wireWatchdog turns "the COM port opened OK at startup" into a live
// connected/disconnected indicator: connState previously stayed frozen at the
// startup message forever, even if the ECU stopped responding entirely
// (unplugged, crashed, brownout) - this derives real liveness from whether
// any serial line has actually been received recently.
func wireWatchdog() {
	for {
		mu.Lock()
		rx := lastRxMs
		mu.Unlock()
		var s string
		switch {
		case rx == 0:
			s = "Dang cho phan hoi... (" + wireInfo + ")"
		case time.Now().UnixMilli()-rx < wireStaleAfter.Milliseconds():
			s = "OK (" + wireInfo + ")"
		default:
			s = fmt.Sprintf("MAT KET NOI - khong phan hoi %ds (%s)", (time.Now().UnixMilli()-rx)/1000, wireInfo)
		}
		mu.Lock()
		connState = s
		mu.Unlock()
		time.Sleep(300 * time.Millisecond)
	}
}

// ---------------- wifi mode ----------------

func wifiGet(path string) (string, error) {
	client := http.Client{Timeout: wifiTimeout}
	resp, err := client.Get(wifiBase + path)
	if err != nil {
		return "", err
	}
	defer resp.Body.Close()
	b, err := io.ReadAll(resp.Body)
	return string(b), err
}

func wifiPoller() {
	lastLogs := ""
	for {
		js, err := wifiGet("/api?act=1")
		if err != nil {
			mu.Lock()
			connState = "MAT KET NOI"
			mu.Unlock()
			time.Sleep(statusPollWiFi)
			continue
		}
		mu.Lock()
		lastStatus = js
		connState = "OK"
		mu.Unlock()
		var obj map[string]interface{}
		if json.Unmarshal([]byte(js), &obj) == nil {
			if lg, ok := obj["logs"].(string); ok && lg != "" && lg != lastLogs {
				s := strings.ReplaceAll(lg, `\n`, " | ")
				if len(s) > 400 {
					s = s[len(s)-400:]
				}
				addTerm("[event-log] " + s)
				lastLogs = lg
			}
		}
		time.Sleep(statusPollWiFi)
	}
}

// ---------------- shared ----------------

func getStatus() string {
	mu.Lock()
	defer mu.Unlock()
	return lastStatus
}

func sendCommand(cmd string) {
	if mode == "wifi" {
		if _, err := wifiGet("/cmd?c=" + url.QueryEscape(cmd)); err != nil {
			addTerm("[bridge] loi gui lenh WiFi: " + err.Error())
		}
	} else if p := curPort(); p != nil {
		p.WriteLine(cmd)
	} else {
		addTerm("[bridge] chua co ket noi - bam 'Ket noi lai'")
	}
}

// reconnect re-establishes the link for either mode: wire re-opens the COM
// port; wifi just clears the state so the poller re-probes immediately (it
// already retries continuously, but this gives the user an explicit action
// and instant feedback).
func reconnect() {
	if mode == "wire" {
		reconnectWire()
	} else {
		mu.Lock()
		connState = "dang ket noi lai..."
		mu.Unlock()
		addTerm("[bridge] thu ket noi lai WiFi " + wifiBase)
	}
}

func termPayload(since int) string {
	mu.Lock()
	defer mu.Unlock()
	total := termBase + len(termLines)
	start := since
	if start < termBase {
		start = termBase
	}
	idx := start - termBase
	var lines []string
	if idx >= 0 && idx <= len(termLines) {
		lines = append(lines, termLines[idx:]...)
	} else {
		lines = append(lines, termLines...)
	}
	out, _ := json.Marshal(map[string]interface{}{
		"mode": mode, "state": connState, "lines": lines, "next": total,
	})
	return string(out)
}

func main() {
	args := os.Args[1:]
	baud := 115200
	httpPort := 8080
	var target string

	if len(args) >= 1 {
		mode = strings.ToLower(args[0])
	}
	if len(args) >= 2 {
		target = args[1]
	}
	if len(args) >= 3 {
		if b, err := strconv.Atoi(args[2]); err == nil {
			baud = b
		}
	}
	if len(args) >= 4 {
		if p, err := strconv.Atoi(args[3]); err == nil {
			httpPort = p
		}
	}

	fmt.Println("=== ECU Bridge GSU ===")
	rd := bufio.NewReader(os.Stdin)
	if mode != "wire" && mode != "wifi" {
		fmt.Print("Kieu ket noi - go 'wire' (day/Serial) hoac 'wifi' [wire]: ")
		s, _ := rd.ReadString('\n')
		s = strings.TrimSpace(strings.ToLower(s))
		if s == "wifi" {
			mode = "wifi"
		} else {
			mode = "wire"
		}
	}

	if mode == "wire" {
		if target == "" {
			fmt.Print("Cong COM (vd COM5): ")
			s, _ := rd.ReadString('\n')
			target = strings.TrimSpace(s)
		}
		var err error
		port, err = openComPort(target, uint32(baud))
		if err != nil {
			fmt.Printf("Loi mo cong %s: %v\n", target, err)
			fmt.Println("Nhan Enter de thoat...")
			rd.ReadString('\n')
			os.Exit(1)
		}
		defer port.Close()
		wireTarget = target
		wireBaud = baud
		wireInfo = fmt.Sprintf("%s @ %d", target, baud)
		connState = "Dang cho phan hoi... (" + wireInfo + ")"
		go wireReader()
		go wirePoller()
		go wireWatchdog()
	} else {
		if target == "" {
			fmt.Print("IP cua ECU [192.168.4.1]: ")
			s, _ := rd.ReadString('\n')
			target = strings.TrimSpace(s)
			if target == "" {
				target = "192.168.4.1"
			}
		}
		wifiBase = "http://" + target
		connState = "dang ket noi..."
		go wifiPoller()
	}

	http.HandleFunc("/", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "text/html; charset=utf-8")
		w.Write(pageHTML)
	})
	http.HandleFunc("/api", func(w http.ResponseWriter, r *http.Request) {
		w.Header().Set("Content-Type", "application/json")
		w.Write([]byte(getStatus()))
	})
	http.HandleFunc("/cmd", func(w http.ResponseWriter, r *http.Request) {
		if c := r.URL.Query().Get("c"); c != "" {
			sendCommand(c)
		}
		w.Header().Set("Content-Type", "text/plain")
		w.Write([]byte("OK"))
	})
	http.HandleFunc("/term", func(w http.ResponseWriter, r *http.Request) {
		since, _ := strconv.Atoi(r.URL.Query().Get("since"))
		w.Header().Set("Content-Type", "application/json")
		w.Write([]byte(termPayload(since)))
	})
	http.HandleFunc("/reconnect", func(w http.ResponseWriter, r *http.Request) {
		reconnect()
		w.Header().Set("Content-Type", "text/plain")
		w.Write([]byte("OK"))
	})

	addr := fmt.Sprintf("127.0.0.1:%d", httpPort)
	fmt.Printf("Che do: %s\n", strings.ToUpper(mode))
	fmt.Printf("Dashboard: http://%s/\n", addr)
	fmt.Println("Dong cua so nay de dung. (Ctrl+C)")
	if err := http.ListenAndServe(addr, nil); err != nil {
		fmt.Println("Loi HTTP server:", err)
		fmt.Println("Nhan Enter de thoat...")
		rd.ReadString('\n')
	}
}

func indexByte(b []byte, c byte) int {
	for i, x := range b {
		if x == c {
			return i
		}
	}
	return -1
}
