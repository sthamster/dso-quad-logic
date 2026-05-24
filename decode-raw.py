#
# Decodes .RAW files into .VCD files
# created by ChatGPT-4o and bring to working state
#
import struct
import sys

def read_varint(stream):
    """Reads a variable-length integer from the given stream."""
    shift = 0
    result = 0
    while True:
        byte = stream.read(1)
        if not byte:
            raise EOFError("Unexpected end of file while reading varint")
        i = ord(byte)
        result |= (i & 0x7f) << shift
        if not (i & 0x80):
            break
        shift += 7
    return result

def decode_raw_file(raw_file):
    """Decodes the raw binary file into a list of signal events."""
    events = []
    with open(raw_file, 'rb') as f:
        while True:
            try:
                varint = read_varint(f)
                duration = varint >> 4
                levels = varint & 0x0F
                events.append((duration, levels))
            except EOFError:
                break
    return events

def write_vcd(events, vcd_file, timescale='1 us', signal_names=['a', 'b', 'c', 'd']):
    """Writes the given events to a VCD file."""
    with open(vcd_file, 'w') as f:
        # Write VCD header
        f.write("$version Converted from LOGICAPP .RAW data $end\n")
        f.write(f"$timescale {timescale} $end\n")
        f.write("$scope module logic $end\n")
        for i, name in enumerate(signal_names):
            f.write(f"$var wire 1 {chr(65 + i)} {name} $end\n")
        f.write("$upscope $end\n")
        f.write("$enddefinitions $end\n")
        
        # Write initial values in $dumpvars
        f.write("$dumpvars ")
        for i, name in enumerate(signal_names):
            f.write(f"0{chr(65 + i)} ")  # Add space between values
        f.write("$end\n")

        # Write events
        current_time = 0
        for duration, levels in events:
            f.write(f"#{current_time} ")
            for i, name in enumerate(signal_names):
                f.write(f"{'1' if (levels >> i) & 1 else '0'}")  # Add space between values
                f.write(f"{chr(65 + i)} ")
            f.write("\n")
            current_time += duration

def convert_raw_to_vcd(raw_file, vcd_file):
    events = decode_raw_file(raw_file)
    write_vcd(events, vcd_file, "2 us", ['ChannelA', 'ChannelB', 'ChannelC', 'ChannelD'])


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 decode-raw.py <raw_file> <vcd_file>")
        sys.exit(1)
    raw_file = sys.argv[1]
    vcd_file = sys.argv[2]
    convert_raw_to_vcd(raw_file, vcd_file)
