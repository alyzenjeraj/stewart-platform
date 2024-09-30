// Select the video element and create a canvas
const videoElement = document.getElementById('video');
const canvas = document.createElement('canvas'); // Hidden canvas to capture frames

// Create a new WebRTC PeerConnection
const peerConnection = new RTCPeerConnection();

// When a new track is received (video stream from server)
peerConnection.ontrack = function(event) {
    console.log('Track received:', event.streams[0]);
    
    // Set the video element's source to the received stream
    videoElement.srcObject = event.streams[0];

    console.log('This is dumb:', event.streams)
    
    
    // Set canvas dimensions to match the video dimensions
    videoElement.onloadedmetadata = function() {
        canvas.width = videoElement.videoWidth;
        canvas.height = videoElement.videoHeight;
    };

    // Start capturing frames every X milliseconds
};


// Log ICE candidate events
peerConnection.onicecandidate = function(event) {
    if (event.candidate) {
        console.log('New ICE candidate:', event.candidate.candidate);
    } else {
        console.log('All ICE candidates have been sent');
    }
};

// Log changes to the ICE connection state
peerConnection.oniceconnectionstatechange = function() {
    console.log('ICE connection state changed:', peerConnection.iceConnectionState);
};

// Add a transceiver for video (even if no local video track is available)
peerConnection.addTransceiver('video', { direction: 'recvonly' });

// Function to start the WebRTC connection
async function startWebRTC() {
    try {
        // Create an SDP offer to start the WebRTC connection
        const offer = await peerConnection.createOffer();
        await peerConnection.setLocalDescription(offer);

        // Send the SDP offer to the Go server via HTTP POST
        const response = await fetch('http://localhost:8080/offer', {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify(peerConnection.localDescription)
        });

        if (!response.ok) {
            throw new Error(`Failed to fetch: ${response.statusText}`);
        }

        // Receive the SDP answer from the server
        const answer = await response.json();
        await peerConnection.setRemoteDescription(new RTCSessionDescription(answer));

        console.log('SDP answer received and applied');
    } catch (error) {
        console.error("Error during WebRTC setup:", error);
    }
}

// Start the WebRTC connection when the page loads
startWebRTC();
