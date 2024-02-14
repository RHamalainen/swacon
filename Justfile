[private]
default:
	@just --list --unsorted

install-dependencies:
	python3.11 -m pip install -r requirements.txt

install:
	pip3.11 install --editable .

test:
	python3.11 -m pytest

lint:
	flake8 . --count --exit-zero --max-complexity=10 --max-line-length=127 --statistics
