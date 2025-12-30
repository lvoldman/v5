def s32(value)-> int: 
    return -(value & 0x80000000) | (value & 0x7fffffff)

list_of_sets = [
    {1, 2, 3},
    {3, 4, 5},
    {5, 6, 7},
    # Add more sets here...
]

# common_elements = set.intersection(*map(set, list_of_sets))
# print(list(common_elements))

import struct, sys

a = 0x3e1e09c3
a = 0xfc0cc307
a = 0x00004040
# b = 0xd97330c3  

b = (a >> 8) & 0x000000FF | (a << 8)  & 0x0000FF00 | (a >> 8) & 0x00FF0000 | (a << 8) & 0xFF000000

print(f'b=0x{b:08x}')

a_h = hex(a)[2:]
b_h = hex(b)[2:]

a_h = '0'*(8-len(a_h)) + a_h
b_h = '0'*(8-len(b_h)) + b_h

# print(f'a = {a}, b= {b}')
print(f'a = {a_h}, b= {b_h}')

a_b = bytes.fromhex(a_h)
b_b = bytes.fromhex(b_h)
print(f'a = {a_b}, b= {b_b}')

a_f = struct.unpack('<f', a_b)
b_f = struct.unpack('<f', b_b)

print(f'a = {a_f}, b= {b_f}')


# sys.exit()

# _float_a = struct.unpack('<f', a_b)[0]
# _float_b = struct.unpack('<f', b_b)[0]

# print(f'a= {a}/{float(a)} /{s32(a)}/{_float_a}  b={a}/{float(a)}/{s32(b)}/{_float_b} ')

# hex_string = '0000803f'  # This is 1.0 in little-endian format

# # Step 1: Convert the hexadecimal string to a byte string
# byte_string = bytes.fromhex(hex_string)

# # Step 2: Convert each byte to a binary string and concatenate
# bit_string = ''.join(format(byte, '08b') for byte in byte_string)

# print(f'bit_string={bit_string}, byte_string = {byte_string}, hex_string= {hex_string} ')