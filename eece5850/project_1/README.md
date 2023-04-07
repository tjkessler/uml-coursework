PROJECT 1
TRAVIS KESSLER, 01425399

A demo for the Python-based implementation of the IDEA encryption/decryption algorithm is available in "demo.ipynb". Source code (pre-processing functions, IDEA even/odd rounds, etc.) is located in "src.py".

Running "demo.ipynb" requires "src.py" to be located in the same directory as "demo.ipynb". Further, Python 3.8+ is needed, as computing the multiplicative inverse requires a function use case not allowed in Python 3.7 or older (pow(a, -1, m)).
