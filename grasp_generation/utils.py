import numpy as np

def mask_to_index(mask):
    idx = []
    for i in range(len(mask)):
        if mask[i] > 0.5:
            idx.append(i)
    return idx

def index_to_mask(idx, mask_len):
    mask = [0] * mask_len

    for i in idx:
        mask[i] = 1
    
    return mask