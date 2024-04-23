package main

import (
	"bytes"
  "context"
  "encoding/binary"
  "reflect"
	"log"
	"math/rand"
	"sync"
	"time"
)

func init() {
	rand.Seed(time.Now().UnixNano())
}

func byteMutator(data []byte, knobs [4]int) bool {
	if knobs == nil {
    // Equal weighting if no knobs provided
		knobs = []int{1, 1, 1, 1} 
	}

	var wg sync.WaitGroup
	results := make(chan bool, 10)

	for i := 0; i < 10; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			if choice(data, knobs) {
				results <- true
			}
		}()
	}

	go func() {
		wg.Wait()
		close(results)
	}()

	for result := range results {
		if result {
			return true
		}
	}
	return false
}

func choice(data []byte, knobs []int) bool {
	totalWeight := 0
	for _, weight := range knobs {
		totalWeight += weight
	}
	randWeight := rand.Intn(totalWeight)

	var fn func([]byte) bool
	weightAccum := 0
	switch {
	case randWeight < weightAccum+knobs[0]:
		fn = bitFlip
	case randWeight < weightAccum+knobs[0]+knobs[1]:
		fn = bytesSwap
	case randWeight < weightAccum+knobs[0]+knobs[1]+knobs[2]:
		fn = byteReplace
	default:
		fn = func(_ []byte) bool { return true }
	}

	return fn(data)
}

func bitFlip(data []byte) bool {
	if len(data) == 0 {
		return false
	}
	byteIndex := rand.Intn(len(data))
	bitIndex := rand.Intn(8)
	bitMask := byte(1 << bitIndex)
	data[byteIndex] ^= bitMask
	return true
}

func bytesSwap(data []byte) bool {
  dLen := len(data)
	if dLen < 2 {
		return false
	}
	idx1, idx2 := rand.Intn(dLen), rand.Intn(dLen)
	if idx1 == idx2 {
		return false
	}
	data[idx1], data[idx2] = data[idx2], data[idx1]
	return true
}

func byteReplace(data []byte) bool {
	if len(data) == 0 {
		return false
	}
	index := rand.Intn(len(data))
	newByte := byte(rand.Intn(256))
	data[index] = newByte
	return true
}

type TypeFmt struct {
	Type  reflect.Kind
	Order binary.ByteOrder
	Size  int
}

var typeFmt = map[string]TypeFmt{
	"uint8":   {reflect.Uint8, binary.LittleEndian, 1},
	"int8":    {reflect.Int8, binary.LittleEndian, 1},
	"int16":   {reflect.Int16, binary.LittleEndian, 2},
	"uint16":  {reflect.Uint16, binary.LittleEndian, 2},
	"int32":   {reflect.Int32, binary.LittleEndian, 4},
	"uint32":  {reflect.Uint32, binary.LittleEndian, 4},
	"int64":   {reflect.Int64, binary.LittleEndian, 8},
	"uint64":  {reflect.Uint64, binary.LittleEndian, 8},
	"float64": {reflect.Float64, binary.LittleEndian, 8},
	"float32": {reflect.Float32, binary.LittleEndian, 4},
	// "string":  {reflect.String, binary.LittleEndian, undefined}, // Strings are handled differently
	// "bool":    {reflect.Bool, binary.LittleEndian, 1}, // Bool is a special case in Go
}

var fieldKnobs []int = []int{1, 1, 1, 27}

func main() {
	data := []byte{0x01, 0x02, 0x03, 0x04}
	log.Println("Original data:", data)
	if byteMutator(data, nil) {
		log.Println("Mutated data:", data)
	} else {
		log.Println("No mutation performed.")
	}
}
