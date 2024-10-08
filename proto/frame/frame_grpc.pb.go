// Code generated by protoc-gen-go-grpc. DO NOT EDIT.
// versions:
// - protoc-gen-go-grpc v1.5.1
// - protoc             v3.12.4
// source: frame/frame.proto

package __

import (
	context "context"
	grpc "google.golang.org/grpc"
	codes "google.golang.org/grpc/codes"
	status "google.golang.org/grpc/status"
)

// This is a compile-time assertion to ensure that this generated file
// is compatible with the grpc package it is being compiled against.
// Requires gRPC-Go v1.64.0 or later.
const _ = grpc.SupportPackageIsVersion9

const (
	FrameService_StreamFrames_FullMethodName = "/FrameService/StreamFrames"
)

// FrameServiceClient is the client API for FrameService service.
//
// For semantics around ctx use and closing/ending streaming RPCs, please refer to https://pkg.go.dev/google.golang.org/grpc/?tab=doc#ClientConn.NewStream.
type FrameServiceClient interface {
	StreamFrames(ctx context.Context, opts ...grpc.CallOption) (grpc.BidiStreamingClient[Frame, Ack], error)
}

type frameServiceClient struct {
	cc grpc.ClientConnInterface
}

func NewFrameServiceClient(cc grpc.ClientConnInterface) FrameServiceClient {
	return &frameServiceClient{cc}
}

func (c *frameServiceClient) StreamFrames(ctx context.Context, opts ...grpc.CallOption) (grpc.BidiStreamingClient[Frame, Ack], error) {
	cOpts := append([]grpc.CallOption{grpc.StaticMethod()}, opts...)
	stream, err := c.cc.NewStream(ctx, &FrameService_ServiceDesc.Streams[0], FrameService_StreamFrames_FullMethodName, cOpts...)
	if err != nil {
		return nil, err
	}
	x := &grpc.GenericClientStream[Frame, Ack]{ClientStream: stream}
	return x, nil
}

// This type alias is provided for backwards compatibility with existing code that references the prior non-generic stream type by name.
type FrameService_StreamFramesClient = grpc.BidiStreamingClient[Frame, Ack]

// FrameServiceServer is the server API for FrameService service.
// All implementations must embed UnimplementedFrameServiceServer
// for forward compatibility.
type FrameServiceServer interface {
	StreamFrames(grpc.BidiStreamingServer[Frame, Ack]) error
	mustEmbedUnimplementedFrameServiceServer()
}

// UnimplementedFrameServiceServer must be embedded to have
// forward compatible implementations.
//
// NOTE: this should be embedded by value instead of pointer to avoid a nil
// pointer dereference when methods are called.
type UnimplementedFrameServiceServer struct{}

func (UnimplementedFrameServiceServer) StreamFrames(grpc.BidiStreamingServer[Frame, Ack]) error {
	return status.Errorf(codes.Unimplemented, "method StreamFrames not implemented")
}
func (UnimplementedFrameServiceServer) mustEmbedUnimplementedFrameServiceServer() {}
func (UnimplementedFrameServiceServer) testEmbeddedByValue()                      {}

// UnsafeFrameServiceServer may be embedded to opt out of forward compatibility for this service.
// Use of this interface is not recommended, as added methods to FrameServiceServer will
// result in compilation errors.
type UnsafeFrameServiceServer interface {
	mustEmbedUnimplementedFrameServiceServer()
}

func RegisterFrameServiceServer(s grpc.ServiceRegistrar, srv FrameServiceServer) {
	// If the following call pancis, it indicates UnimplementedFrameServiceServer was
	// embedded by pointer and is nil.  This will cause panics if an
	// unimplemented method is ever invoked, so we test this at initialization
	// time to prevent it from happening at runtime later due to I/O.
	if t, ok := srv.(interface{ testEmbeddedByValue() }); ok {
		t.testEmbeddedByValue()
	}
	s.RegisterService(&FrameService_ServiceDesc, srv)
}

func _FrameService_StreamFrames_Handler(srv interface{}, stream grpc.ServerStream) error {
	return srv.(FrameServiceServer).StreamFrames(&grpc.GenericServerStream[Frame, Ack]{ServerStream: stream})
}

// This type alias is provided for backwards compatibility with existing code that references the prior non-generic stream type by name.
type FrameService_StreamFramesServer = grpc.BidiStreamingServer[Frame, Ack]

// FrameService_ServiceDesc is the grpc.ServiceDesc for FrameService service.
// It's only intended for direct use with grpc.RegisterService,
// and not to be introspected or modified (even as a copy)
var FrameService_ServiceDesc = grpc.ServiceDesc{
	ServiceName: "FrameService",
	HandlerType: (*FrameServiceServer)(nil),
	Methods:     []grpc.MethodDesc{},
	Streams: []grpc.StreamDesc{
		{
			StreamName:    "StreamFrames",
			Handler:       _FrameService_StreamFrames_Handler,
			ServerStreams: true,
			ClientStreams: true,
		},
	},
	Metadata: "frame/frame.proto",
}
