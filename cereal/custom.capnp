using Cxx = import "./include/c++.capnp";
$Cxx.namespace("cereal");

@0xb526ba661d550a59;

# custom.capnp: a home for empty structs reserved for custom forks
# These structs are guaranteed to remain reserved and empty in mainline
# cereal, so use these if you want custom events in your fork.

# you can rename the struct, but don't change the identifier
struct CustomReserved0 @0x81c2f05a394cf4af {
}

struct CustomReserved1 @0xaedffd8f31e7b55d {
}

struct CustomReserved2 @0xf35cc4560bbf6ec2 {
}

struct CustomReserved3 @0xda96579883444c35 {
}

struct CustomReserved4 @0x80ae746ee2596b11 {
}

struct CustomReserved5 @0xa5cd762cd951a455 {
}

struct CustomReserved6 @0xf98d843bfd7004a3 {
}

struct CustomReserved7 @0xb86e6369214c01c8 {
}

struct CustomReserved8 @0xf416ec09499d9d19 {
}

struct CustomReserved9 @0xa1680744031fdb2d {
}

struct LiveMapData {
  speedLimit @0 :Float32;
  speedLimitValid @1 :Bool;
  speedLimitAhead @2 :Float32;
  speedLimitAheadDistance @3 :Float32;
  speedLimitAheadValid @4 :Bool;
  turnSpeedLimit @5 :Float32;
  turnSpeedLimitValid @6 :Bool;
  turnSpeedLimitSign @7 :Int16;
  turnSpeedLimitEndDistance @8 :Float32;
  distToTurn @9 :Float32;
  mapValid @10 :Bool;
  roadName @11 :Text;
  gpsAccuracy @12 :Float32;
  ts @13 :Float64;
  wayId @14 :UInt64;
  currentRoadName @15 :Text;
}