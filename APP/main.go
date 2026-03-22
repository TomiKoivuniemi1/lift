package main

import (
	"bufio"
	"context"
	"os"
	"os/signal"
	"runtime"
	"strings"
	"syscall"

	"go.bug.st/serial"

	"github.com/go-ole/go-ole"
	"github.com/go-ole/go-ole/oleutil"
)

func main() {
	if runtime.GOOS != "windows" {
		return
	}

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	_ = ole.CoInitialize(0)
	defer ole.CoUninitialize()

	u, err := oleutil.CreateObject("SAPI.SpVoice")
	if err != nil {
		return
	}
	defer u.Release()

	v, err := u.QueryInterface(ole.IID_IDispatch)
	if err != nil {
		return
	}
	defer v.Release()

	p, err := serial.Open("COM3", &serial.Mode{BaudRate: 9600})
	if err != nil {
		return
	}
	defer p.Close()

	go func() { <-ctx.Done(); _ = p.Close() }()

	r := bufio.NewReader(p)
	for ctx.Err() == nil {
		s, err := r.ReadString('\n')
		if err != nil {
			return
		}

		s = strings.TrimSpace(s)
		if !strings.HasPrefix(s, "EVENT ") {
			continue
		}

		parts := strings.Fields(s)
		if len(parts) < 2 {
			continue
		}

		tag := parts[1]

		const (
			SPF_ASYNC            = 1
			SPF_PURGEBEFORESPEAK = 2
		)

		say := ""
		flags := SPF_ASYNC

		switch tag {
		case "MOVE_TO":
			if len(parts) >= 3 && len(parts[2]) == 1 && parts[2][0] >= '0' && parts[2][0] <= '3' {
				say = "Moving to floor " + parts[2]
				flags |= SPF_PURGEBEFORESPEAK
			}
		case "ARRIVED":
			if len(parts) >= 3 && len(parts[2]) == 1 && parts[2][0] >= '0' && parts[2][0] <= '3' {
				say = "Arrived at floor " + parts[2]
			}
		case "DOORS_OPEN":
			say = "Doors opening"
		case "DOORS_CLOSED":
			say = "Doors closing"
		case "DIR_UP":
			say = "Going up"
		case "DIR_DOWN":
			say = "Going down"
		default:
			// ignore unknown events
		}

		if say == "" {
			continue
		}

		_, _ = oleutil.CallMethod(v, "Speak", say, flags)
	}
}
