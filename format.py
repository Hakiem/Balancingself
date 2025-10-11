# format.py
Import("env")
import os

def run_format():
    print("Running clang-format on source files...")
    files = []
    for root, _, fs in os.walk("src"):
        files += [os.path.join(root, f) for f in fs if f.endswith((".c", ".h"))]
    for root, _, fs in os.walk("include"):
        files += [os.path.join(root, f) for f in fs if f.endswith((".c", ".h"))]
    for f in files:
        print(f"Formatting {f}")
        os.system(f"clang-format -i -style=file {f}")
    print("Formatting complete!")

run_format()