import numpy as np

def prob_to_log_odds(p):
    return np.log(p / (1 - p))

def log_odds_to_prob(l):
    return 1 - 1 / (1 + np.exp(l))

def inverse_sensor_model(hit: bool, l_occ=0.85, l_free=-0.4):
    return prob_to_log_odds(l_occ) if hit else prob_to_log_odds(l_free)
