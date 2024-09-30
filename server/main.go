package main

import (
    "fmt"
    "log"
    "net"
    "net/http"
    "sync"

    pb "stewart-platform/proto/frame"  // Import your frame proto package
    "google.golang.org/grpc"
    "github.com/gorilla/websocket"
)

// Server struct to store the latest frame
type Server struct {
    pb.UnimplementedFrameServiceServer
    latestFrame []byte
    mu          sync.Mutex
}

var upgrader = websocket.Upgrader{
    CheckOrigin: func(r *http.Request) bool {
        return true
    },
}

// StreamFrames function that receives frames via gRPC
func (s *Server) StreamFrames(stream pb.FrameService_StreamFramesServer) error {
    for {
        frame, err := stream.Recv()
        if err != nil {
            return err
        }

        // Store the latest frame in the server
        s.mu.Lock()
        s.latestFrame = frame.Data
        s.mu.Unlock()

        fmt.Printf("Received frame of size: %d bytes\n", len(frame.Data))
    }
}

// WebSocket handler for streaming the frames
func (s *Server) handleWebSocket(w http.ResponseWriter, r *http.Request) {
    conn, err := upgrader.Upgrade(w, r, nil)
    if err != nil {
        log.Println("Error upgrading to WebSocket:", err)
        return
    }
    defer conn.Close()

    for {
        s.mu.Lock()
        frame := s.latestFrame
        s.mu.Unlock()

        if len(frame) > 0 {
            err = conn.WriteMessage(websocket.BinaryMessage, frame)
            if err != nil {
                log.Println("Error sending frame:", err)
                break
            }
        }
    }
}

// Serve the HTML page for the frontend interface
func (s *Server) serveHTML(w http.ResponseWriter, r *http.Request) {
    http.ServeFile(w, r, "../client/index.html")
}

func main() {
    server := &Server{}

    // Start the gRPC server for receiving frames
    go func() {
        lis, err := net.Listen("tcp", ":5051")  // gRPC listening on port 5051
        if err != nil {
            log.Fatalf("Failed to listen: %v", err)
        }
        grpcServer := grpc.NewServer()
        pb.RegisterFrameServiceServer(grpcServer, server)

        if err := grpcServer.Serve(lis); err != nil {
            log.Fatalf("Failed to serve: %v", err)
        }
    }()

    // Serve the WebSocket for video streaming and the HTML interface
    http.HandleFunc("/ws", server.handleWebSocket)  // WebSocket endpoint
    http.HandleFunc("/", server.serveHTML)          // Serve HTML page

    log.Println("Serving WebSocket video stream on ws://localhost:8080/ws")
    log.Fatal(http.ListenAndServe(":8080", nil))  // Serve everything on port 8080
}
