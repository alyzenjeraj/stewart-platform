package main

import (
    "encoding/json" // Add this import
    "fmt"
    "net/http"

    "github.com/pion/webrtc/v4"
)

func main() {
    // Create a new WebRTC API instance
    peerConnection, err := webrtc.NewPeerConnection(webrtc.Configuration{})
    if err != nil {
        panic(err)
    }

    // Create a new video track for the webcam stream
    videoTrack, err := webrtc.NewTrackLocalStaticSample(webrtc.RTPCodecCapability{
        MimeType: webrtc.MimeTypeVP8,
    }, "video", "webrtc")
    if err != nil {
        panic(err)
    }

    // Add the track to the peer connection
    _, err = peerConnection.AddTrack(videoTrack)
    if err != nil {
        panic(err)
    }

    // Create a simple HTTP handler for SDP (WebRTC signaling)
    http.HandleFunc("/offer", func(w http.ResponseWriter, r *http.Request) {
        offer := webrtc.SessionDescription{}

        // Decode the incoming offer (SDP)
        if err := json.NewDecoder(r.Body).Decode(&offer); err != nil {
            panic(err)
        }

        // Set remote description
        if err := peerConnection.SetRemoteDescription(offer); err != nil {
            panic(err)
        }

        // Create and set local description (answer)
        answer, err := peerConnection.CreateAnswer(nil)
        if err != nil {
            panic(err)
        }

        if err := peerConnection.SetLocalDescription(answer); err != nil {
            panic(err)
        }

        // Encode the answer to JSON and send it back
        w.Header().Set("Content-Type", "application/json")
        if err := json.NewEncoder(w).Encode(peerConnection.LocalDescription()); err != nil {
            panic(err)
        }
    })

    // Serve static files (HTML + JavaScript)
    http.Handle("/", http.FileServer(http.Dir("./static")))

    fmt.Println("WebRTC server is running on :8080")
    http.ListenAndServe(":8080", nil)
}
