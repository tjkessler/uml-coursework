import hashlib


def hash_file(filename_in: str, filename_out: str):
    """ hash_file: hashes a file using SHA-384

    Args:
        filename_in (str): name of file to hash
        filename_out (str): name of hashed file
    """

    with open(filename_in, 'rb') as f:
        data = f.read()
    f.close()
    hash = bytes(hashlib.sha384(data).hexdigest(), 'utf8')
    with open(filename_out, 'wb') as f:
        f.write(hash)
    f.close()


def hash_bytes(b: bytes) -> str:
    """ hash_bytes: hashes bytes using SHA-384

    Args:
        b (bytes): bytes to hash

    Returns:
        str: hashed bytes, hexadecimal format
    """

    return hashlib.sha384(b).hexdigest()
