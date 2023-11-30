# NOTE: This module assumes Python 2 and not Python 3
# NOTE: This module assumes an existing installation of 'python-mnist'
# NOTE: This module assumes the mnist training / test data located at './datasets/MNIST'

# Team 1 members:
# Lemanov, Mack, Waiblinger, Weber, Wilke


import os, time, numpy as np
from numpy.random import uniform
from mnist import MNIST


# Multi-Layer Perceptron -----------------------------------------------------

class Perceptron():
    def __init__(self, layers=None, input_size=None):
        self.layers = layers if layers is not None else []
        self.weights = [np.array([[]]) for _ in self.layers]
        self.biases = [np.array([]) for _ in self.layers]
        self.input_size = input_size
        self.loss = None
        self.lr = 1

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
        self.lr = lr


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


    def train(self, x, y, epochs=1):
        print("\nTraining -------------------------------")
        start = time.time()
        for e in range(epochs):
            print("Epoch {}: ".format(e + 1)),
            progress = ''
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

                    self.weights[i] = self.weights[i] - self.lr * dL_dW
                    self.biases[i] = self.biases[i] - self.lr * dL_dB

                    dL_dA = np.dot(dL_dA * dA_dC, self.weights[i])

                delete = "\b" * (len(progress) + 1)
                progress = "{:.1f}%".format(len(predictions) / float(len(y)) * 100)
                print(delete + progress),

            print(" Loss={:.3f}".format(self.loss(predictions, y)))

        end = time.time()
        finish = "Finished after {:.2f}s ".format(end - start)
        print(finish + "-" * (40 - len(finish)) + "\n")


    def load_weights(self, filename='parameters.npz'):
        data = np.load(filename, allow_pickle=True)
        self.weights = data['arr_0'][0]
        self.biases = data['arr_0'][1]

    
    def save_weights(self, filename='parameters'):
        data = [np.array(self.weights), np.array(self.biases)]
        np.savez_compressed(filename, data)


class Dense():
    def __init__(self, units, a_func):
        self.num_units = units
        self.a_func = a_func


# Cerebellar Model Articulation Controller -----------------------------------

class CMAC():
    def __init__(self, size_out=1, res=10, rf=3, fill=1, lr=0.01, opt=None):
        def _init_weights():
            weights = np.zeros((size_out, res, res))
            for out in range(size_out):
                if type(fill) == np.ndarray:
                    weights[out] = fill
                else:
                    num_neurons = int(res * res * fill)
                    while (num_neurons > 0):
                        x, y = uniform(0, res, 2).astype(int)
                        if weights[out, x, y] == 0:
                            weights[out, x, y] = uniform(0, 1)
                            num_neurons -= 1
            return weights

        self.resolution = res
        self.rf = rf
        self.weights = _init_weights()
        self.loss = mean_squared_error#mean_error
        self.lr = lr
        self.optimizer = opt

        self._xs = None
        self._ys = None
    

    def predict(self, x, save=False):
        def _quantized_field(x):
            # main index in weights
            x_q = (x * self.resolution + 0.5).astype(int)
            xs, ys = [], [] # indices of neurons within receptive field
            offsets = [0]
            for x in range(1, self.rf):
                offsets.append(x)
                offsets.append(-x)

            for offset in offsets:
                if (len(xs) < self.rf):
                    tmp = x_q[0] + offset # next index in weights
                    if tmp >= 0 and tmp < self.resolution: xs.append(tmp)
                if (len(ys) < self.rf):
                    tmp = x_q[1] + offset
                    if tmp >= 0 and tmp < self.resolution: ys.append(tmp)

            return xs, ys
        
        xs, ys = _quantized_field(x)

        result = np.zeros(self.weights.shape[0])

        for i, weights in enumerate(self.weights): # for each output
            for x in xs: # for each index in receptive field
                for y in ys:
                    result[i] += weights[x, y] # some weights might be zero

        if save:
            self._xs = xs
            self._ys = ys

        return result


    def train(self, x, y, epochs=1):
        print("\nTraining -------------------------------")
        start = time.time()
        if self.optimizer is not None:
            self.optimizer.start_training(self)

        for e in range(epochs):
            print("Epoch {}: ".format(e + 1)),
            predictions = []
            for sample, target in zip(x, y):
                # perform forward pass
                predictions.append(self.predict(sample, save=True))

                # perform backward pass
                l = self.loss(predictions[-1], target) # error signal
                n = 0
                for i in self._xs:
                    for j in self._ys:
                        n += 1 if self.weights[0, i, j] != 0 else 0

                if n == 0: continue # avoid divide by zero

                for i in range(len(target)):
                    for j in self._xs:
                        for k in self._ys:
                            if self.weights[i, j, k] == 0: continue
                            # NOTE using "mean_error" yields better results,
                            # but "-=" has to exchanged with "+="
                            self.weights[i, j, k] -= self.lr * l / float(n)

            loss = self.loss(predictions, y)
            print("Loss={:.3f}".format(np.abs(loss)))

            if self.optimizer is not None:
                # optimizer decides to stop
                if self.optimizer.optimize(self, e, loss): break

        end = time.time()
        finish = "Finished after {:.2f}s ".format(end - start)
        print(finish + "-" * (40 - len(finish)) + "\n")


    def load_weights(self, filename='cmac_parameters.npz'):
        self.weights = np.load(filename)['arr_0']


    def save_weights(self, filename='cmac_parameters'):
        np.savez_compressed(filename, self.weights)


# Optimizer ------------------------------------------------------------------

class Felix():
    def __init__(self, start_lr, end_lr, epochs, early_stop=True):
        self.slr = start_lr
        self.elr = end_lr
        self.max_epochs = epochs
        self.early_stop = early_stop
        self.best_loss = float('inf')

    
    def start_training(self, model):
        model.lr = self.slr


    def optimize(self, model, epoch, loss):
        model.lr = np.linspace(self.slr, self.elr, self.max_epochs)[epoch]
        if not self.early_stop: return False
        if np.abs(loss) < self.best_loss:
            self.best_loss = np.abs(loss)
            return False
        else:
            return True


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


def mean_error(prediction, target, derivative=False):
    result = (target - prediction).mean()
    if derivative:
        result = -1 * np.ones(prediction.shape)
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
    
    correct = 0
    for p, t in zip(idx_predictions, idx_targets):
        if p == t: correct += 1

    return float(correct) / len(predictions)


def one_hot(y, num_classes):
    targets = np.zeros((len(y), num_classes))
    for i, t in enumerate(y):
        targets[i][int(t)] = 1
    return targets


def split_dataset(x, y, split=0.1):
    train_mask = [True if uniform(0, 1) > split else False for _ in y]
    test_mask = [not train for train in train_mask]

    return x[train_mask], y[train_mask], x[test_mask], y[test_mask]


# Example usage --------------------------------------------------------------

def get_joint_dataset(path='train_data_norm.csv', split=0.1):
    data = np.loadtxt(path, delimiter=',', skiprows=1)
    samples = data[:, [2, 3]]
    targets = data[:, [0, 1]]

    return split_dataset(samples, targets, split=split)


def get_mnist_dataset(split='both'):
    mnist = MNIST("datasets/MNIST")

    if split in ['train', 'both']:
        x_train, y_train = mnist.load_training()
        x_train = np.asarray(x_train).astype(np.float32)[:20000]
        y_train = np.asarray(y_train).astype(np.float32)[:20000]
        y_train = one_hot(y_train, 10)
    
    if split in ['test', 'both']:
        x_test, y_test = mnist.load_testing()
        x_test = np.asarray(x_test).astype(np.float32)
        y_test = np.asarray(y_test).astype(np.float32)
        y_test = one_hot(y_test, 10)
    
    if split == 'train': return x_train, y_train
    elif split == 'test': return x_test, y_test
    else: return x_train, y_train, x_test, y_test


def mlp_joints():
    mlp = Perceptron(input_size=2)
    mlp.add_layer(Dense(units=16, a_func=linear)) # input layer
    mlp.add_layer(Dense(units=2, a_func=linear)) # output layer
    mlp.compile(loss=mean_squared_error, lr=0.002)

    x_train, y_train, x_test, y_test = get_joint_dataset()
    params = 'mlp_joint_params'

    print("=" * 79)
    print("Using MLP to predict joint values")

    if os.path.exists(params + '.npz'):
        mlp.load_weights(filename=params + '.npz')
    else:
        predictions = [mlp.predict(x) for x in x_test]
        print("\nMSE before training: {:.3f}"
            .format(mean_squared_error(predictions, y_test)))
        
        mlp.train(x=x_train, y=y_train, epochs=10)
        mlp.save_weights(filename=params)

    predictions = [mlp.predict(x) for x in x_test]
    print("MSE after training: {:.3f}\n"
          .format(mean_squared_error(predictions, y_test)))

    # generate a dataset with truth values from the MLP
    # samples = uniform(0, 1, (10000, 2))
    # targets = [mlp.predict(s) for s in samples]
    # np.savetxt('train_data_mlp_gen.csv',
    #            X=np.concatenate((targets, samples), axis=1),
    #            delimiter=',',
    #            header='LShoulderPitch,LShoulderRoll,BlobX,BlobY')


def cmac_joints():
    EPOCHS = 30

    grid = np.zeros((50, 50))       # neuron grid
    grid[0::5, 0::5] = 1            # 1 0 0 0 0 1 ... 0
    grid[1::5, 1::5] = 1            # 0 1 0 0 0 0 ... 0
    grid[2::5, 2::5] = 1            # ...
    grid[3::5, 3::5] = 1            # ...
    grid[4::5, 4::5] = 1            # 0 0 0 0 1 0 ... 1
    grid *= uniform(0, 1, (50, 50))

    optimizer = Felix(start_lr=0.05, end_lr=0.005, epochs=EPOCHS)
    cmac = CMAC(size_out=2, res=50, rf=5, fill=grid, opt=optimizer)

    x_train, y_train, x_test, y_test = get_joint_dataset()
    params = 'cmac_joint_params'

    print("=" * 79)
    print("Using CMAC to predict joint values")
    
    if os.path.exists(params + '.npz'):
        cmac.load_weights(params + '.npz')
    else:
        predictions = [cmac.predict(x) for x in x_test]
        print("\nMSE before training: {:.2f}"
            .format(np.abs(mean_error(predictions, y_test))))

        cmac.train(x=x_train, y=y_train, epochs=EPOCHS)
        cmac.save_weights(params)

    predictions = [cmac.predict(x) for x in x_test]
    print("MSE after training: {:.2f}\n"
          .format(np.abs(mean_error(predictions, y_test))))


def mlp_mnist():
    mlp = Perceptron(input_size=784)
    mlp.add_layer(Dense(units=784, a_func=sigmoid)) # input layer
    mlp.add_layer(Dense(units=128, a_func=sigmoid)) # hidden layer
    mlp.add_layer(Dense(units=32, a_func=sigmoid)) # hidden layer
    mlp.add_layer(Dense(units=10, a_func=softmax)) # output layer
    mlp.compile(loss=categorical_crossentropy, lr=0.01)

    params = 'mlp_mnist_params'

    print("=" * 79)
    print("Using MLP to predict MNIST labels")

    if os.path.exists(params + '.npz'):
        x_test, y_test = get_mnist_dataset(split='test')
        mlp.load_weights(params + '.npz')

    else:
        # very slow call
        x_train, y_train, x_test, y_test = get_mnist_dataset(split='both')

        predictions = [mlp.predict(x) for x in x_test]
        print("Acurracy before training: {:.2f}%"
              .format(accuracy(predictions, y_test) * 100))

        mlp.train(x=x_train, y=y_train, epochs=3)
        mlp.save_weights(params)

    predictions = [mlp.predict(x) for x in x_test]
    print("Acurracy after training: {:.2f}%"
          .format(accuracy(predictions, y_test) * 100))


if __name__ == '__main__':
    # mlp_mnist() # NOTE this call will trigger (slow) training on mnist

    # mlp_joints()

    cmac_joints()
