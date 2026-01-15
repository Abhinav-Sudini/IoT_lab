import serial
import time
import sys

from datetime import datetime



# CONFIGURATION
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200

def create_new_csv(csv_ind):
    with open(f"data_{csv_ind}.csv","w") as fd:
        fd.write("temperature,humidity,pressure,gas,time_stamp_esp,time_stamp_pc\n")



try:
    # Initialize Serial Connection
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")
    print("Press Ctrl+C to stop.")
    print("-" * 30)

    # Allow time for the connection to stabilize
    time.sleep(2)


    csv_file_index = 1
    num_lines_in_csv = 0
    create_new_csv(csv_file_index)

    while True:
        # Check if data is waiting in the buffer
        if ser.in_waiting > 0:
            try:
                # Read a line, decode bytes to string, remove whitespace/newlines
                line = ser.readline().decode('utf-8', errors='ignore').strip()
                
                # Only print if the line is not empty
                prefix = "Data :"
                if (not line) or (not line.startswith(prefix)) :
                    continue

                clean = line.removeprefix(prefix)
                with open(f"data_{csv_file_index}.csv", "a") as f:
                    f.write(clean +f",{datetime.now().isoformat()}"+ "\n")

                # print(line)

                num_lines_in_csv += 1
                if (num_lines_in_csv > 2000):
                    print("done for :",csv_file_index)
                    csv_file_index += 1
                    num_lines_in_csv = 0
                    create_new_csv(csv_file_index)
            
            except UnicodeDecodeError:
                # Sometimes startup garbage data causes decode errors; ignore them
                pass

except serial.SerialException:
    print(f"Error: Could not open {SERIAL_PORT}. Make sure the Arduino IDE Serial Monitor is CLOSED.")
except KeyboardInterrupt:
    print("\nExiting script...")
finally:
    # Close the connection nicely
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Serial port closed.")