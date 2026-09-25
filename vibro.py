# AI Generated code for real-time vibration monitoring and ESC control.

# The on-disk burst format (little-endian):
#   magic "DVB1" | version u16 | axes u16 | sample_rate u32 |
#   sample_count u32 | timestamp_ns u64 | sequence u64 | int16 samples[count][3]

import argparse
import sys

from PyQt6.QtWidgets import QApplication, QDialog

import app
import dialogs
import utils


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="VibroApp")
    parser.add_argument(
        "--file", metavar="PATH",
        help="open a binary burst file (DVB1) instead of asking for a mode",
    )
    parser.add_argument(
        "--debug", action="store_true",
        help="debug mode: validate the file (with --file) and exit without GUI",
    )
    args = parser.parse_args()

    # Debug mode: just check the file and report, no GUI
    if args.debug:
        if not args.file:
            print("ERROR: --debug requires --file PATH")
            sys.exit(2)
        try:
            bursts = utils.parse_dvb1_file(args.file)
        except Exception as error:
            print(f"ERROR: could not open {args.file}: {error}")
            sys.exit(1)
        if not bursts:
            print(f"ERROR: no DVB1 bursts found in {args.file}")
            sys.exit(1)
        for index, burst in enumerate(bursts):
            print(
                f"Burst {index + 1}: seq={burst['sequence']} "
                f"rate={burst['sample_rate']}Hz samples={burst['sample_count']} "
                f"ts_ns={burst['timestamp_ns']}"
            )
        print(f"OK: {len(bursts)} burst(s) loaded from {args.file}")
        sys.exit(0)

    application = QApplication(sys.argv)
    port = None
    demo_mode = False
    file_path = None

    if args.file:
        # CLI file mode: skip the mode dialog
        file_path = args.file
    else:
        # Show mode selection dialog at startup
        mode_dialog = dialogs.ModeDialog()
        if mode_dialog.exec() != QDialog.DialogCode.Accepted:
            sys.exit(0)
        if mode_dialog.mode == dialogs.ModeDialog.MODE_DEMO:
            demo_mode = True
        elif mode_dialog.mode == dialogs.ModeDialog.MODE_FILE:
            file_path = mode_dialog.file_path
        else:
            port = mode_dialog.port

    win = app.RealtimeVibeApp(port, demo_mode=demo_mode, file_path=file_path)
    win.show()
    sys.exit(application.exec())
