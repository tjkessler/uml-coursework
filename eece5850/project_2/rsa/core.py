from typing import Tuple


def encrypt_hex(m: str, key: Tuple[int]) -> str:
    """ encrypt_hex: encrypts a hexadecimal message with supplied key such
    that:

    $ c = m^e mod n $

    Args:
        m (str): message to encrypt, hexadecimal format
        key (Tuple[int]): public key (e, n)

    Returns:
        str: encrypted message, hexadecimal format
    """

    e, n = key
    return hex(pow(int(m, 16), e, n))


def decrypt_hex(c: str, key: Tuple[int]) -> str:
    """ decrypt_hex: decrypts a hexadecimal message with supplied key such
    that:

    $ m = c^d mod n $

    Args:
        c (str): message to decrypt, hexadecimal format
        key (Tuple[int]): private key (d, n)

    Returns:
        str: decrypted message, hexadecimal format
    """

    d, n = key
    return hex(pow(int(c, 16), d, n))[2:]


def sign_hex(m: str, key: Tuple[int]) -> str:
    """ sign_hex: signs a hexadecimal message using supplied key such that:

    $ s = m^d mod n $

    Args:
        m (str): message to sign, hexadecimal format
        key (Tuple[int]): private key (d, n)

    Returns:
        str: decrypted message, hexadecimal format
    """

    d, n = key
    return hex(pow(int(m, 16), d, n))
