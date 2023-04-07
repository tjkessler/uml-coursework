from typing import List, Tuple


def bitlist_to_int(bitlist: List[int]) -> int:
    """ Converts a list of bits (zeros and ones) to an integer

    e.g., [1, 0, 0, 0] -> 8

    Args:
        bitlist (List[int]): list of bits

    Returns:
        int: resulting integer
    """

    out = 0
    for bit in bitlist:
        out = (out << 1) | bit
    return out


def int_to_bitlist(val: int, n_bits: int) -> List[int]:
    """ Converts an integer into a list of bits

    e.g., 8, 4 bits -> [1, 0, 0, 0]

    Args:
        val (int): number to convert to list of bits
        n_bits (int): number of bits in bitlist

    Returns:
        List[int]: list of bits
    """

    return [1 if val & (1 << (n_bits - 1 - n)) else 0 for n in range(n_bits)]


def str_to_bitlist(s: str) -> List[int]:
    """ Converts a string to a list of bits

    Args:
        s (str): string to convert

    Returns:
        List[int]: string, represented as a bitlist
    """

    result = []
    for c in s:
        bits = bin(ord(c))[2:]
        bits = '00000000'[len(bits):] + bits
        result.extend([int(b) for b in bits])
    return result


def format_message(msg: str) -> List[int]:
    """ First transforms a string into a 64-bit integers, then each 64-bit
    integer sequentially into four 16-bit integers; list length will always
    satisfy `length % 4 || length % 16 || length % 64 == 0` due to zero-padding

    Args:
        msg (str): message to format

    Returns:
        List[int]: list of 16-bit integers
    """

    def _segment_block(block: List[int]) -> List[int]:
        """ binary list length 64 -> four 16-bit integers """
        return [
            bitlist_to_int(block[:16]),
            bitlist_to_int(block[16:32]),
            bitlist_to_int(block[32:48]),
            bitlist_to_int(block[48:])
        ]

    bits = str_to_bitlist(msg)
    ints16 = []
    for i in range(len(bits) // 64):
        _base = i * 64
        block = []
        for j in range(64):
            block.append(bits[_base + j])
        ints16.extend(_segment_block(block))
    if len(bits) % 64 != 0:
        final_block = bits[-(len(bits) % 64):]
        for _ in range(64 - len(final_block)):
            final_block.append(0)
        ints16.extend(_segment_block(final_block))
    return ints16


def expand_key(key: int) -> Tuple[List[int]]:
    """ Expands a 128-bit binary integer into 52 16-bit integers,
    for both encryption and decryption

    Args:
        key (int): 128-bit key

    Returns:
        Tuple(List[int]): (encryption key list of 52 16-bit integers,
            decryption key list of 52 16-bit integers)
    """

    key_bits = int_to_bitlist(key, 128)
    key_bits.extend(key_bits)
    enc_key = []
    for i in range(52):
        base = i // 8 * 25 % 128
        offset = i % 8 * 16
        enc_key.append(bitlist_to_int(
            key_bits[base + offset: base + offset + 16]
        ))

    dec_key = [0 for _ in range(52)]
    dec_key = [
        pow(enc_key[48], -1, 65537),
        -1 * enc_key[50],
        -1 * enc_key[49],
        pow(enc_key[51], -1, 65537)
    ]
    dec_key[0] = pow(enc_key[48], -1, 65537)
    dec_key[1] = -1 * enc_key[50]
    dec_key[2] = -1 * enc_key[49]
    dec_key[3] = pow(enc_key[51], -1, 65537)
    for i in range(8):
        dec_key.append(enc_key[6 * (7 - i) + 4])
        dec_key.append(enc_key[6 * (7 - i) + 5])
        dec_key.append(pow(enc_key[6 * (7 - i) + 0], -1, 65537))
        dec_key.append(-1 * enc_key[6 * (7 - i) + 2])
        dec_key.append(-1 * enc_key[6 * (7 - i) + 1])
        dec_key.append(pow(enc_key[6 * (7 - i) + 3], -1, 65537))

    return (enc_key, dec_key)


def bit_mul(a: int, b: int) -> int:
    """ Multiplies two 16-bit integers

    Args:
        a (int): first 16-bit integer
        b (int): second 16-bit integer

    Returns:
        int: resulting 16-bit integer
    """

    return (a * b) % 65537


def bit_add(a: int, b: int) -> int:
    """ Adds two 16-bit integers, throwing away carry

    Args:
        a (int): first 16-bit integer
        b (int): second 16-bit integer

    Returns:
        int: resulting 16-bit integer
    """

    return (a + b) % 65536


def odd_round(xa: int, xb: int, xc: int, xd: int,
              ka: int, kb: int, kc: int, kd: int) -> Tuple[int]:
    """ Performs an odd round for IDEA encryption

    Args:
        xa (int): first 16-bit component of round message
        xb (int): second 16-bit component of round message
        xc (int): third 16-bit component of round message
        xd (int): fourth 16-bit component of round message
        ka (int): first 16-bit expanded key
        kb (int): second 16-bit expanded key
        kc (int): third 16-bit expanded key
        kd (int): fourth 16-bit expanded key

    Returns:
        Tuple[int]: tuple with four elements, xa', xb', xc', xd'
    """

    return (
        bit_mul(xa, ka),
        bit_add(xc, kc),
        bit_add(xb, kb),
        bit_mul(xd, kd)
    )


def even_round(xa: int, xb: int, xc: int, xd: int,
               ke: int, kf: int) -> Tuple[int]:
    """ Performs an even round for IDEA encryption

    Args:
        xa (int): first 16-bit component of round message
        xb (int): second 16-bit component of round message
        xc (int): third 16-bit component of round message
        xd (int): fourth 16-bit component of round message
        ke (int): first 16-bit expanded key
        kf (int): second 16-bit expanded key

    Returns:
        Tuple[int]: tuple with four elements, xa', xb', xc', xd'
    """

    yin = xa ^ xb
    zin = xc ^ xd

    yout = bit_mul(bit_add(bit_mul(ke, yin), zin), kf)
    zout = bit_add(bit_mul(ke, yin), yout)

    return (
        xa ^ yout,
        xb ^ yout,
        xc ^ zout,
        xd ^ zout
    )


def idea_crypt_block(xa: int, xb: int, xc: int, xd: int,
                     crypt_key: List[int]) -> Tuple[int]:
    """ Encrypts/decrypts a 64-bit integer using IDEA

    Args:
        xa (int): first 16-bit component of message
        xb (int): second 16-bit component of message
        xc (int): third 16-bit component of message
        xd (int): fourth 16-bit component of message
        crypt_key (List[int]): 52 16-bit keys, either encryption or
            decryption key

    Returns:
        Tuple[int]: encrypted/decrypted xa', xb', xc', xd'
    """

    xa, xb, xc, xd = odd_round(
        xa, xb, xc, xd,
        crypt_key[0],
        crypt_key[2],
        crypt_key[1],
        crypt_key[3]
    )

    _key_idx = 4
    for _ in range(8):
        xa, xb, xc, xd = even_round(
            xa, xb, xc, xd,
            crypt_key[_key_idx],
            crypt_key[_key_idx + 1]
        )
        xa, xb, xc, xd = odd_round(
            xa, xb, xc, xd,
            crypt_key[_key_idx + 2],
            crypt_key[_key_idx + 4],
            crypt_key[_key_idx + 3],
            crypt_key[_key_idx + 5]
        )
        _key_idx += 6

    return (xa, xb, xc, xd)
