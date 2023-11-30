# NOTE: This module assumes Python 2 and not Python 3
# NOTE: This module assumes an existing installation of 'python-mnist'
# NOTE: This module assumes the mnist training / test data located at './datasets/MNIST'

# Team 1 members:
# Lemanov, Mack, Waiblinger, Weber, Wilke

import os, time, numpy as np
from numpy.random import uniform
from mnist import MNIST


# Multi-layer perceptron -----------------------------------------------------

class Perceptron():
    def __init__(self, layers=None, input_size=None):
        self.layers = layers if layers is not None else []
        self.weights = [np.array([[]]) for _ in self.layers]
        self.biases = [np.array([]) for _ in self.layers]
        self.input_size = input_size
        self.loss = None
        self.learning_rate = 1

        self._x = [] # x
        self._c = [] # w * x + b
        self._a = [] # a_func(_c)
        self._p = [] # a_func'(_c)


    def add_layer(self, layer):
        self.layers.append(layer)
        inp = self.layers[-2].num_units if len(self.layers) > 1 else self.input_size

        # TODO initialization should depend on layer size
        w_l = np.array([[uniform(-1, 1) for _ in range(inp)] for _ in range(layer.num_units)])
        b_l = np.array([0] * layer.num_units)

        self.weights.append(w_l)
        self.biases.append(b_l)


    def compile(self, loss, lr):
        self.loss = loss
        self.learning_rate = lr


    def predict(self, x, save=False):
        for i, layer in enumerate(self.layers):
            c = np.dot(self.weights[i], x) + self.biases[i]
            a = layer.a_func(c)
            if save:
                p = layer.a_func(c, derivative=True)
                self._x.append(x)
                self._c.append(c)
                self._a.append(a)
                self._p.append(p)
            x = a
        return x


    def train(self, x, y, epochs=1, verbose=True):
        def _print_clear():
            print ""
            return 0

        def _print_training(p_l):
            delete = "\b" * p_l

            index = len(predictions) - 1
            current_loss = self.loss(predictions[-1], y[index])
            progress = (float)(len(predictions)) / len(y) * 100

            out = "Loss={:.3f}, Progress={:.2f}%".format(current_loss, progress)

            print delete,
            print out,

            return len(out)+2

        print_length = _print_clear()
        print("Training -----------------------------")
        start = time.time()
        for e in range(epochs):
            print "Epoch "+str(e+1)+":",
            predictions = []
            for sample, target in zip(x, y):
                # perform forward pass
                predictions.append(self.predict(sample, save=True))

                # perform backward pass
                dL_dA = self.loss(predictions[-1], target, derivative=True)
                for i in range(-1, -(1+len(self.layers)), -1):
                    dA_dC = self._p[i]
                    dC_dW = self._x[i]
                    dL_dW = np.outer(dL_dA * dA_dC, dC_dW)
                    dL_dB = dL_dA * dA_dC * 1

                    self.weights[i] = self.weights[i] - self.learning_rate * dL_dW
                    self.biases[i] = self.biases[i] - self.learning_rate * dL_dB

                    dL_dA = np.dot(dL_dA * dA_dC, self.weights[i])

                if verbose: print_length = _print_training(print_length)

            if verbose: print_length = _print_clear()

        end = time.time()
        print "Finished after {:.2f}s ---------------\n".format(end - start)


    def load_weights(self, filename='parameters.npz'):
        data = np.load(filename, allow_pickle=True)
        self.weights = data['arr_0'][0]
        self.biases = data['arr_0'][1]

    
    def save_weights(self, filename='parameters'):
        data = [np.array(self.weights), np.array(self.biases)]
        np.savez_compressed(filename, data)


# Fully-connected layer ------------------------------------------------------

class Dense():
    def __init__(self, units, a_func):
        self.num_units = units
        self.a_func = a_func


# Activation functions -------------------------------------------------------

def sigmoid(x, derivative=False):
    pos_mask = (x >= 0)
    neg_mask = (x < 0)

    z = np.zeros_like(x)
    z[pos_mask] = np.exp(-x[pos_mask])
    z[neg_mask] = np.exp(x[neg_mask])

    dividend = np.ones_like(x)
    dividend[neg_mask] = z[neg_mask]

    result = dividend / (1 + z)

    if derivative: result = result * (1 - result)
    return result


def softmax(x, derivative=False):
    x_shift = x - max(x)
    result = np.exp(x_shift) / np.sum(np.exp(x_shift))
    # TODO fix derivative of softmax
    if derivative: result = np.ones(x.shape)
    return result


def linear(x, derivative=False):
    return np.ones(x.shape) if derivative else x


def mean_squared_error(prediction, target, derivative=False):
    result = np.square(target - prediction).mean()
    if derivative: result = 2 * (prediction - target) / len(target)
    return result


def mean_absolute_error(prediction, target, derivative=False):
    result = np.abs(prediction - target).mean()
    if derivative:
        result = np.ones(prediction.shape)
        result[prediction == target] = 0
        result[prediction < target] = -1
    return result


def categorical_crossentropy(prediction, target, derivative=False):
    result = -np.sum(target * np.log(prediction))
    # TODO fix derivative of cc
    if derivative: result = prediction - target
    return result


# Utility functions ----------------------------------------------------------

def accuracy(predictions, targets):
    idx_predictions = np.array([list(p).index(p.max()) for p in predictions])
    idx_targets = np.array([list(t).index(t.max()) for t in targets])
    correct = sum([1 for p, t in zip(idx_predictions, idx_targets) if p == t])

    return correct / (float)(len(predictions))


def one_hot(y, num_classes):
    targets = np.zeros((len(y), num_classes))
    for i, t in enumerate(y):
        targets[i][(int)(t)] = 1
    return targets


def split_dataset(x, y, split=0.1):
    train_mask = [True if uniform(0, 1) > split else False for _ in range(len(x))]
    test_mask = [not train for train in train_mask]

    x_train = x[train_mask]
    y_train = y[train_mask]

    x_test = x[test_mask]
    y_test = y[test_mask]

    return x_train, y_train, x_test, y_test


# Example usage --------------------------------------------------------------

def get_joint_mlp(loss, lr):
    mlp = Perceptron(input_size=2)
    mlp.add_layer(Dense(units=16, a_func=linear)) # input layer
    mlp.add_layer(Dense(units=2, a_func=linear)) # output layer
    mlp.compile(loss=loss, lr=lr)
    return mlp


def get_joint_dataset():
    # load csv file
    path = "train_Data_norm.csv"
    data = np.loadtxt(path, delimiter=';', skiprows=1)

    # collect samples and targets
    targets = data[:, [0, 1]]
    samples = data[:, [2, 3]]

    # return training and test set
    return split_dataset(samples, targets, split=0.2)


def get_mnist_mlp(loss, lr):
    mlp = Perceptron(input_size=784)
    mlp.add_layer(Dense(units=784, a_func=sigmoid)) # input layer
    mlp.add_layer(Dense(units=128, a_func=sigmoid)) # hidden layer
    mlp.add_layer(Dense(units=32, a_func=sigmoid)) # hidden layer
    mlp.add_layer(Dense(units=10, a_func=softmax)) # output layer
    mlp.compile(loss=loss, lr=lr)
    return mlp


def get_mnist_dataset(split='both', train_subset=60000, test_subset=10000):
    mnist = MNIST("datasets/MNIST")

    if split in ['train', 'both']:
        x_train, y_train = mnist.load_training()
        x_train = np.asarray(x_train).astype(np.float32)[:train_subset]
        y_train = np.asarray(y_train).astype(np.float32)[:train_subset]
        y_train = one_hot(y_train, 10)
    
    if split in ['test', 'both']:
        x_test, y_test = mnist.load_testing()
        x_test = np.asarray(x_test).astype(np.float32)[:test_subset]
        y_test = np.asarray(y_test).astype(np.float32)[:test_subset]
        y_test = one_hot(y_test, 10)
    
    if split == 'train': return x_train, y_train
    elif split == 'test': return x_test, y_test
    else: return x_train, y_train, x_test, y_test


def regress_joints():
    LOSS_JOINT = mean_absolute_error
    LR_JOINT = 0.001
    EPOCHS_JOINT = 10
    LIMIT_LOWS = [-2, -0.31]
    LIMIT_HIGHS = [2, 1.32]

    def denormalize(prediction, lows, highs):
        result = []
        for i in range(len(prediction)):
            size = (highs[i] - lows[i])
            result.append(prediction[i] * size - size * 0.5)
        return result

    x_train, y_train, x_test, y_test = get_joint_dataset()
    mlp = get_joint_mlp(LOSS_JOINT, LR_JOINT)

    if not os.path.exists('joint_parameters.npz'):
        predictions = [mlp.predict(x) for x in x_test]
        print("MAE on test set before training: {:.4f}".format(LOSS_JOINT(predictions, y_test)))

        mlp.train(x=x_train, y=y_train, epochs=EPOCHS_JOINT)
        mlp.save_weights(filename='joint_parameters')
    else:
        mlp.load_weights(filename='joint_parameters.npz')

    predictions = [mlp.predict(x) for x in x_test]
    print("MAE on test set after training: {:.4f}".format(LOSS_JOINT(predictions, y_test)))

    # actual prediction for joint angle values
    return [denormalize(p, LIMIT_LOWS, LIMIT_HIGHS) for p in predictions]


def classify_mnist():
    LOSS_MNIST = categorical_crossentropy
    LR_MNIST = 0.01
    EPOCHS_MNIST = 3

    mlp = get_mnist_mlp(LOSS_MNIST, LR_MNIST)

    if not os.path.exists('mnist_parameters.npz'):
        x_train, y_train, x_test, y_test = get_mnist_dataset(split='both', train_subset=20000)

        # before training
        predictions = [mlp.predict(sample) for sample in x_test]
        print("Acurracy before training (10000 test samples): "+str(accuracy(predictions, y_test) * 100)+"%")

        mlp.train(x=x_train, y=y_train, epochs=EPOCHS_MNIST)
        mlp.save_weights(filename='mnist_parameters')
    else:
        x_test, y_test = get_mnist_dataset(split='test')
        mlp.load_weights(filename='mnist_parameters.npz')

    # after training
    predictions = [mlp.predict(sample) for sample in x_test]
    print("Acurracy after training (10000 test samples): "+str(accuracy(predictions, y_test) * 100)+"%")


if __name__ == '__main__':
    classify_mnist()

    regress_joints()
