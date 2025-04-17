import subprocess
import pandas as pd
import sys
import os
import datetime
import re

def parse_log_lines_to_dicts(file_bytes):
    decoded = file_bytes.decode('utf-8', errors='replace')
    lines = decoded.splitlines()

    gps_data = []
    pattern = r'(\w+)\s*:\s*([^\s,{}]+)'

    for line in lines:
        if "GPS" not in line:
            continue
        kv_pairs = re.findall(pattern, line)
        if kv_pairs:
            gps_data.append({key: value for key, value in kv_pairs})

    if not gps_data:
        print("❌ No GPS lines could be parsed into key-value pairs.")
        return None

    return pd.DataFrame(gps_data)

def convert_bin_to_csv(bin_file_path):
    timestamp_str = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    base_name = os.path.splitext(os.path.basename(bin_file_path))[0]
    output_file_name = f"{base_name}_parsed_{timestamp_str}.csv"

    print(f"Converting {bin_file_path} to {output_file_name}...")

    try:
        result = subprocess.run(
            ['mavlogdump.py', '--types', 'GPS', bin_file_path],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=10
        )

        print("📤 mavlogdump stdout (first 200 chars):")
        print(result.stdout.decode(errors='replace')[:200])
        print("📥 mavlogdump stderr (first 200 chars):")
        print(result.stderr.decode(errors='replace')[:200])

        df = parse_log_lines_to_dicts(result.stdout)

        if df is None:
            print("⚠️  Trying full log fallback...")
            fallback = subprocess.run(
                ['mavlogdump.py', bin_file_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=10
            )
            df = parse_log_lines_to_dicts(fallback.stdout)

        if df is None:
            print("❌ Still no usable GPS data.")
            return

        df.to_csv(output_file_name, index=False)
        print(f"📦 File '{output_file_name}' saved. Size: {os.path.getsize(output_file_name)} bytes")
        print("✅ Parsed GPS DataFrame:")
        print(df.head())

        if 'Lat' in df.columns and 'Lng' in df.columns:
            print("✅ GPS data present!")
            print("Latitude (first 5):", df['Lat'].head())
            print("Longitude (first 5):", df['Lng'].head())
        else:
            print("⚠️ GPS fields not found, check column names:", df.columns)

    except Exception as e:
        print("❌ Error running mavlogdump or parsing:", e)

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python3 CSVtojulia.py <BIN_FILE>")
    else:
        convert_bin_to_csv(sys.argv[1])

