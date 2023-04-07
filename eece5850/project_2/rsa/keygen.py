from typing import Tuple
from random import randrange, randint
from math import gcd


def n_bit_prime(n: int) -> int:
    """ n_bit_prime: generates a prime integer in the range

    $ [2^{n-1} + 1, 2^n - 1] $

    NOTE: exponential time complexity

    Args:
        n (int): number of bits

    Returns:
        int: prime integer
    """

    while True:
        p = randrange(2**(n - 1) + 1, 2**n - 1, 2)
        if all(p % i != 0 for i in range(3, int((p ** 0.5) + 1), 2)):
            return p


def n_bit_rand(n: int) -> int:
    """ n_bit_rand: generates a random integer in the range

    $ [2^{n-1} + 1, 2^n - 1] $

    Args:
        n (int): number of bits

    Returns:
        int: random integer
    """

    return randint(2**(n - 1) + 1, 2**n - 1)


def generate_key(n_bits: int = 32) -> Tuple[Tuple[int], Tuple[int]]:
    """ generate_key: generates a public key and private key for RSA
    encryption/decryption

    Args:
        n_bits (int, default 32): number of bits for primes P and Q;
            must be >= 4

    Returns:
        Tuple[Tuple[int], Tuple[int]]: (public key, private key) i.e.,
            ((e, n), (d, n))
    """

    # Prime generation algorithm only works with 4-bit or larger integers
    if n_bits < 4:
        raise ValueError(f'Must be at least 4 bits! n_bits: {n_bits}')

    # Generate primes `p` and `q`
    p = n_bit_prime(n_bits)
    q = n_bit_prime(n_bits)

    # Make sure they're not the same
    while p == q:
        q = n_bit_prime(n_bits)

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
