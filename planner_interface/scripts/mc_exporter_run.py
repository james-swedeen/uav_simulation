#!/usr/bin/env python3

from pathlib import Path
import argparse
from planner_interface.mc_exporter import MCExporter

def main(args=None):
  """Initialize a plotter and run it"""
  exporter = MCExporter(args.data_dir)
  exporter.run()


if __name__ == "__main__":
  # Get data directory from user in order to run
  parser = argparse.ArgumentParser()
  parser.add_argument("data_dir", type=str, help="Directory storing MC data")
  args = parser.parse_args()
  main(args)

