a = "$00000000000000000000000000000000000000000000000000000000000000!"
b = "$00000000000000000000000000000000000000000000000000000000000000!"
hex_values_a = [ord(c) for c in a]
hex_values_b = [ord(c) for c in b]

# Convert it to bytearray
bytes_a = bytearray(hex_values_a)
bytes_b = bytearray(hex_values_b)

# # Convert it integer
integer_a = int.from_bytes(bytes_a, byteorder='little')
integer_b = int.from_bytes(bytes_b, byteorder='little')

c = integer_a + integer_b

print(c.to_bytes(64, 'little'))