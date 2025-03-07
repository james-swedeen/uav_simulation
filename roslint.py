#! /usr/bin/env python3
# Use this file to run automatic Python linting of the user-created files in a ros2 package

import os
from glob import glob


class tcolor:
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'

ignore_list = ['setup.py', '__init__.py', 'test_copyright.py', 'test_flake8.py', 'test_pep257.py', 'roslint.py']

# Get all python files in project
g = glob('./**/*.py', recursive=True)
to_do: list[str] = []
for idx, item in enumerate(g):
    if not any([item.split('/')[-1] in ignored for ignored in ignore_list]):

        to_do.append(item[2:])

print(f"Found {len(to_do)} to lint...")

print(f"{tcolor.OKCYAN}Starting {tcolor.BOLD}isort{tcolor.ENDC} {tcolor.OKCYAN}linting...{tcolor.ENDC}")
for name in to_do:
    print(f"{tcolor.OKBLUE}Running isort on {name}{tcolor.ENDC}")
    os.system(f"python3 -m isort {name}")

print(f"{tcolor.OKCYAN}Starting {tcolor.BOLD}pylint{tcolor.ENDC} {tcolor.OKCYAN}linting...{tcolor.ENDC}")
for name in to_do:
    print(f"{tcolor.OKBLUE}Running pylint on {name}{tcolor.ENDC}")
    os.system(f"python3 -m pylint --rcfile .pylintrc {name}")

print(f"{tcolor.OKCYAN}Starting {tcolor.BOLD}mypy{tcolor.ENDC} {tcolor.OKCYAN}linting...{tcolor.ENDC}")
for name in to_do:
    print(f"{tcolor.OKBLUE}Running mypy on {name}{tcolor.ENDC}")
    os.system(f"python3 -m mypy {name}")
