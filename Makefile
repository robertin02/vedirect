install:
	python3 -m venv venv
	./.venv/bin/pip3 install -r requirements.txt

clean:
	rm -rf venv
	find . -type d -name "__pycache__" -exec rm -rf {} +
freeze:
	./.venv/bin/pip3 freeze > requirements.txt
