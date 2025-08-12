from webserver import convert_to_8bit_hex


high, low = convert_to_8bit_hex(hex(25))

print(low)
print(high)
