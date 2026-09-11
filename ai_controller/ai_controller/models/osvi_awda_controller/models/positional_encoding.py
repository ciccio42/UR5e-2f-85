"""Positional encodings used by OSVI-AWDA's inverse model."""

import math

import numpy as np
import torch
import torch.nn as nn


class PositionalEncoding(nn.Module):
    """Copy of ``archs/PositionalEncoding.py::PositionalEncoding``."""

    def __init__(self, d_model, dropout=0.1, max_len=5000, time_dim=1):
        super().__init__()
        self.dropout = nn.Dropout(p=dropout)
        self.time_dim = time_dim

        pe = torch.zeros(max_len, d_model)
        position = torch.arange(0, max_len, dtype=torch.float).unsqueeze(1)
        div_term = torch.exp(
            torch.arange(0, d_model, 2).float()
            * (-math.log(10000.0) / d_model)
        )
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        self.register_buffer("pe", pe)

    def forward(self, x):
        x = x + self.pe[: x.size(self.time_dim)]
        return self.dropout(x)


class TemporalPositionalEncoding(nn.Module):
    """Copy of ``archs/PositionalEncoding.py::TemporalPositionalEncoding``."""

    def __init__(self, d_model, dropout=0.1, max_len=5000):
        super().__init__()
        self.dropout = nn.Dropout(p=dropout)

        pe = torch.zeros(max_len, d_model)
        position = torch.arange(0, max_len, dtype=torch.float).unsqueeze(1)
        div_term = torch.exp(
            torch.arange(0, d_model, 2).float()
            * (-np.log(10000.0) / d_model)
        )
        pe[:, 0::2] = torch.sin(position * div_term)
        pe[:, 1::2] = torch.cos(position * div_term)
        pe = pe.unsqueeze(0).transpose(1, 2)
        self.register_buffer("pe", pe)

    def forward(self, x):
        if len(x.shape) < 3:
            raise ValueError("x requires at least 3 dims: (B, C, ...)")
        old_shape = x.shape
        x = x.reshape((x.shape[0], x.shape[1], -1))
        x = x + self.pe[:, :, : x.shape[-1]]
        return self.dropout(x).reshape(old_shape)
