package main

import (
	"fmt"
	"io"
	"net/http"
	"os"
	"path/filepath"
	"sync"
	"time"
)

var csv_index = 0
var csv_len = 0
var fs_mutex sync.Mutex

var lastping time.Time = time.Now()

//	func new_fd() error {
//		if err := file_dis.Close(); err != nil {
//				log.Fatal(err)
//		}
//		csv_index += 1
//		csv_len = 0
//		var err error
//		file_dis,err =  os.OpenFile(fmt.Sprintf("data/csv_data_%d.csv",csv_index), os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0644)
//		return err
//	}
func nowISO() string {
	return time.Now().UTC().Format(time.RFC3339Nano)
}
func is_active() string {
	time_dif := time.Since(lastping)
	// fmt.Println(time.Now(),time_dif,lastping)
	if time_dif.Minutes() < 5.0 {
		return fmt.Sprintf("<hr><h2 style=\"color: #2fff05;\">Server Active</h2><h3>last ping : %v UST</h3>", lastping.Format("2006-01-02 15:04:05"))
	} else {
		return fmt.Sprintf("<hr><h2 style=\"color: #ff0516;\">Server InActive</h2><h3>last ping : %v</h3>", lastping.Format("2006-01-02 15:04:05"))
	}
}
func apiHandler(w http.ResponseWriter, req *http.Request) {
	if req.Method == "POST" {

		bodyBytes, err := io.ReadAll(req.Body)
		defer req.Body.Close() // Essential: close the body when the function returns
		if err != nil {
			http.Error(w, "can't read body", http.StatusBadRequest)
			return
		}
		if len(bodyBytes) > 0 && bodyBytes[len(bodyBytes)-1] == '\n' {
			bodyBytes[len(bodyBytes)-1] = ','
		} else {
			bodyBytes = append(bodyBytes, ',')
		}
		bodyBytes = append(bodyBytes, []byte(nowISO())...)
		bodyBytes = append(bodyBytes, '\n')
		fs_mutex.Lock()
		defer fs_mutex.Unlock()

		file_dis, err := os.OpenFile(fmt.Sprintf("data/csv_data_%d.csv", csv_index), os.O_APPEND|os.O_CREATE|os.O_WRONLY, 0644)
		if err != nil {
			http.Error(w, "cant open file", http.StatusInternalServerError)
		}

		if _, err := file_dis.Write(bodyBytes); err != nil {
			http.Error(w, "can't append file", http.StatusInternalServerError)
			return
		}
		csv_len += 1
		if csv_len > 2000 {
			csv_index += 1
			csv_len = 0
		}

		fmt.Fprint(w, "done")
		lastping = time.Now()

	} else {
		http.Error(w, "sad", http.StatusBadRequest)
	}
}
func rootHandler(w http.ResponseWriter, req *http.Request) {
	data_dir := "./data/"
	file := filepath.Join(data_dir, req.URL.Path)
	fs_mutex.Lock()
	defer fs_mutex.Unlock()
	w.Header().Set("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
	w.Header().Set("Pragma", "no-cache")
	w.Header().Set("Expires", "0")
	http.ServeFile(w, req, file)
	if req.URL.Path == "/" {
		fmt.Println("/")
		fmt.Fprint(w, is_active())
	}
}
func main() {
	http.HandleFunc("/api/", apiHandler)
	http.HandleFunc("/", rootHandler)
	fmt.Println("server running ")
	http.ListenAndServe(":4440", nil)
}
