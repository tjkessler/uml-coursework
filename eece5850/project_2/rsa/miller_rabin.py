from random import randrange
from typing import Tuple
from math import gcd

from .keygen import n_bit_rand


def miller_rabin(n: int, k: int) -> bool:
    """ miller_rabin: Miller-Rabin test for primality, from:
    http://cs.indstate.edu/~rpavuluru/Abstract.pdf

    Args:
        n (int): number to test
        k (int): number of tests to perform

    Returns:
        bool: True if prime, else False
    """

    if n == 2:
        return True

    if n % 2 == 0:
        return False

    r, s = 0, n - 1
    while s % 2 == 0:
        r += 1
        s //= 2
    for _ in range(k):
        a = randrange(2, n - 1)
        x = pow(a, s, n)
        if x == 1 or x == n - 1:
            continue
        for _ in range(r - 1):
            x = pow(x, 2, n)
            if x == n - 1:
                break
        else:
            return False
    return True


def n_bit_prime(n: int, k: int) -> int:
    """ n_bit_prime: generates a prime integer in the range

    $ [2^{n-1} + 1, 2^n - 1] $

    Uses miller-rabin prime test

    Args:
        n (int): number of bits
        k (int): number of tests for miller-rabin prime test

    Returns:
        int: prime integer
    """

    p = randrange(2**(n - 1) + 1, 2**n - 1, 2)
    while not miller_rabin(p, k):
        p = randrange(2**(n - 1) + 1, 2**n - 1, 2)
    return p


def generate_key(n_bits: int = 32,
                 k: int = 40) -> Tuple[Tuple[int], Tuple[int]]:
    """ generate_key: generates a public key and private key for RSA
    encryption/decryption

    Args:
        n_bits (int, default 32): number of bits for primes P and Q;
            must be >= 4
        k (int, default 40): number of tests for Miller-Rabin prime generation

    Returns:
        Tuple[Tuple[int], Tuple[int]]: (public key, private key) i.e.,
            ((e, n), (d, n))
    """

    # Prime generation algorithm only works with 4-bit or larger integers
    if n_bits < 4:
        raise ValueError(f'Must be at least 4 bits! n_bits: {n_bits}')

    # Generate primes `p` and `q`
    p = n_bit_prime(n_bits, k)
    q = n_bit_prime(n_bits, k)

    # Make sure they're not the same
    while p == q:
        q = n_bit_prime(n_bits, k)

    # Compute `n` and `p_n`
    n = p * q
    p_n = (p - 1) * (q - 1)

    # Find int `e` that's coprime with `p_n`
    e = n_bit_rand(n_bits)
    while(gcd(e, p_n) != 1):
        e = n_bit_rand(n_bits)

    # Multiplicative inverse to get `d`
    d = pow(e, -1, p_n)

    # Return (public key, private key)
    return ((e, n), (d, n))
