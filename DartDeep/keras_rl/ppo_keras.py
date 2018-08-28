from keras.models import Sequential, Model
from keras.layers import Dense, Activation, Flatten, Input, Concatenate
from keras.optimizers import Adam, SGD
from keras.backend import normal


def get_policy_network(num_states, num_actions):
    _policy = Sequential()
    _policy.add(Dense(1024, activation='relu', input_dim=num_states))
    _policy.add(Dense(512, activation='relu'))
    _policy.add(Dense(num_actions, activation='linear'))

    return _policy


def get_value_network(num_states):
    _value = Sequential()
    _value.add(Dense(1024, activation='relu', input_dim=num_states))
    _value.add(Dense(512, activation='relu'))
    _value.add(Dense(1, activation='linear'))

    return _value

policy_network = get_policy_network(3, 1)
value_network = get_value_network(3)

policy_optimizer = SGD(lr=5e-5, momentum=0.9)
value_optimizer = SGD(lr=1e-2, momentum=0.9)

policy_network.compile(optimizer=policy_optimizer,
                       loss='mse')

print(policy_network.summary())
print(value_network.summary())
