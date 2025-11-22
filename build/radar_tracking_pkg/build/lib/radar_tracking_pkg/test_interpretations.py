#!/usr/bin/env python3
import struct

# Your actual measurements
TARGET_1 = 1.4  # door
TARGET_2 = 5.0  # whiteboard
TARGET_3 = 2.7  # you

# Sample data from earlier
samples = [
    bytes.fromhex("07 D0 00 00 1F FF"),  # Sample 1
    bytes.fromhex("00 1B 00 00 09 15"),  # Sample 4
    bytes.fromhex("04 0F 1C 23 09 00"),  # Sample 5
    bytes.fromhex("FF 8D AF 00 00 20"),  # Sample 3
]

print("Looking for values that match 1.4m, 5.0m, or 2.7m...")
print()

for idx, data in enumerate(samples):
    print(f"Sample {idx+1}: {' '.join(f'{b:02X}' for b in data)}")
    
    # Try all possible 2-byte combinations with different scales
    tests = []
    
    # Bytes [0:2]
    le = struct.unpack('<H', data[0:2])[0]
    be = struct.unpack('>H', data[0:2])[0]
    tests.append(("Bytes[0:2] LE *0.1", le * 0.1))
    tests.append(("Bytes[0:2] BE *0.1", be * 0.1))
    tests.append(("Bytes[0:2] LE *0.01", le * 0.01))
    tests.append(("Bytes[0:2] BE *0.01", be * 0.01))
    
    # Bytes [2:4]
    le2 = struct.unpack('<H', data[2:4])[0]
    be2 = struct.unpack('>H', data[2:4])[0]
    tests.append(("Bytes[2:4] LE *0.1", le2 * 0.1))
    tests.append(("Bytes[2:4] BE *0.1", be2 * 0.1))
    
    # Bytes [4:6]
    le3 = struct.unpack('<H', data[4:6])[0]
    be3 = struct.unpack('>H', data[4:6])[0]
    tests.append(("Bytes[4:6] LE *0.1", le3 * 0.1))
    tests.append(("Bytes[4:6] BE *0.1", be3 * 0.1))
    
    # Single bytes
    tests.append(("Byte[0] *0.1", data[0] * 0.1))
    tests.append(("Byte[1] *0.1", data[1] * 0.1))
    
    for name, value in tests:
        # Check if close to any target (within 0.5m)
        if abs(value - TARGET_1) < 0.5:
            print(f"  ✓✓✓ {name}: {value:.2f}m (DOOR 1.4m!)")
        elif abs(value - TARGET_2) < 0.5:
            print(f"  ✓✓✓ {name}: {value:.2f}m (WHITEBOARD 5m!)")
        elif abs(value - TARGET_3) < 0.3:
            print(f"  ✓ {name}: {value:.2f}m (YOU 2.7m)")
        elif 0.5 <= value <= 50:
            print(f"      {name}: {value:.2f}m")
    
    print()
