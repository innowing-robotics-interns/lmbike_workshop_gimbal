"""Small Gaussian-process Bayesian optimizer (expected improvement, minimization)."""

from __future__ import annotations

import numpy as np
from scipy.stats import norm


def _rbf(x: np.ndarray, y: np.ndarray, length: float, variance: float) -> np.ndarray:
    d2 = np.sum((x[:, None, :] - y[None, :, :]) ** 2, axis=-1)
    return variance * np.exp(-0.5 * d2 / (length ** 2))


def expected_improvement(mu: np.ndarray, std: np.ndarray, best: float) -> np.ndarray:
    std = np.maximum(std, 1e-9)
    z = (best - mu) / std
    return (best - mu) * norm.cdf(z) + std * norm.pdf(z)


class BayesOptimizer:
    """Minimize a black-box over the unit hypercube [0, 1]^d."""

    def __init__(self, dim: int, *, length: float = 0.25, noise: float = 0.05, seed: int = 0) -> None:
        self.dim = int(dim)
        self.length = float(length)
        self.noise = float(noise)
        self.rng = np.random.default_rng(seed)
        self.x = np.zeros((0, self.dim))
        self.y = np.zeros((0,))

    def tell(self, x: np.ndarray, y: float) -> None:
        self.x = np.vstack([self.x, np.asarray(x, dtype=float).reshape(1, -1)])
        self.y = np.append(self.y, float(y))

    def ask(self, n_candidates: int = 256) -> np.ndarray:
        if len(self.y) == 0:
            return self.rng.random(self.dim)
        cand = self.rng.random((n_candidates, self.dim))
        mu, std = self._posterior(cand)
        best = float(np.min(self.y))
        scores = expected_improvement(mu, std, best)
        return cand[int(np.argmax(scores))]

    def best(self) -> tuple[np.ndarray, float]:
        i = int(np.argmin(self.y))
        return self.x[i].copy(), float(self.y[i])

    def _posterior(self, x_star: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        y = self.y - np.mean(self.y)
        variance = float(np.var(self.y)) + 1e-6
        k = _rbf(self.x, self.x, self.length, variance) + (self.noise * variance) * np.eye(len(self.y))
        ks = _rbf(self.x, x_star, self.length, variance)
        kss = _rbf(x_star, x_star, self.length, variance)
        chol = np.linalg.cholesky(k + 1e-8 * np.eye(len(self.y)))
        alpha = np.linalg.solve(chol.T, np.linalg.solve(chol, y))
        mu = ks.T @ alpha + np.mean(self.y)
        v = np.linalg.solve(chol, ks)
        var = np.maximum(np.diag(kss - v.T @ v), 1e-12)
        return mu, np.sqrt(var)
