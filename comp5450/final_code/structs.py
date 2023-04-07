from typing import Tuple, List, Dict

import numpy
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.utils.data import DataLoader, Dataset


class SoftmaxMLP(nn.Module):

    def __init__(self, input_dim: int, hidden_dim: int, output_dim: int):
        """
        Feed-forward multilayer perceptron with softmax layer at output

        Args:
            input_dim (int): number of input features for model
            hidden_dim (int): number of neurons in hidden layers
            n_hidden (int): number of hidden layers
            output_dim (int): number of features per sample target
            dropout (float): random neuron dropout probability during training
        """

        super(SoftmaxMLP, self).__init__()
        self.hidden1 = nn.Linear(input_dim, hidden_dim)
        self.hidden2 = nn.Linear(hidden_dim, hidden_dim)
        self.output = nn.Linear(hidden_dim, output_dim)
        self.sigmoid = nn.ReLU()
        self.softmax = nn.Softmax(dim=1)
        

    def forward(self, x: 'torch.tensor') -> 'torch.tensor':
        """
        Feed-forward operation

        Args:
            x (torch.tensor): input data, size [m, n_features]

        Returns:
            torch.tensor: fed-forward data, size [m, n_targets]
        """

        x = self.hidden1(x)
        x = self.sigmoid(x)
        x = self.hidden2(x)
        x = self.sigmoid(x)
        x = self.output(x)
        x = self.softmax(x)
        return x

class PMDataset(Dataset):

    def __init__(self, X: 'numpy.array', y: 'numpy.array'):
        """
        Formats data into struct for batching/training

        Args:
            X (numpy.array): input variables, shape [n_samples, n_features]
            y (numpy.array): target variables, shape [n_samples, n_targets]
        """

        if len(X) != len(y):
            raise ValueError('Dim 0 of `X` and `y` must match: {}, {}'.format(
                len(X), len(y)
            ))

        self.X = torch.as_tensor(X).type(torch.float32)
        self.y = torch.as_tensor(y).type(torch.float32)

    def __len__(self) -> int:
        """
        Returns:
            int: n_samples (length of supplied `X`)
        """

        return len(self.X)

    def __getitem__(self, idx: int) -> Dict[str, 'torch.tensor']:
        """
        Args:
            idx (int): index to query

        Returns:
            Dict[str, torch.tensor]: {'X': torch.tensor, 'y': torch.tensor}
                for given index
        """

        return {
            'X': self.X[idx],
            'y': self.y[idx]
        }


def train_model(model: 'SoftmaxMLP', dataset: 'PMDataset',
                epochs: int = 100, batch_size: int = 1, verbose: int = 0,
                **kwargs) -> Tuple['SoftmaxMLP', List[float]]:
    """
    Trains a model using supplied data

    Args:
        model (SoftmaxMLP): model to train
        dataset (PMDataset): dataset used for training
        epochs (int, optional): number of training epochs (iterations)
            (default: 100)
        batch_size (int, optional): size of each training batch (default: 1)
        verbose (int, optional): if >0, prints loss every `this` epochs
            (default: 0, no printing)
        **kwargs (optional): additional arguments to be passed to
            torch.optim.SGD()

    Returns:
        Tuple[SoftmaxMLP, List[float]]: (trained model, training losses)
    """

    dataloader_train = DataLoader(dataset, batch_size=batch_size, shuffle=True)

    opt = torch.optim.Adam(model.parameters(), **kwargs)
    loss = nn.BCELoss()

    model.train()
    train_losses = []
    for epoch in range(epochs):

        train_loss = 0.0

        for batch in dataloader_train:

            opt.zero_grad()
            pred = model(batch['X'])
            target = batch['y']
            _loss_val = loss(pred, target)
            _loss_val.backward()
            opt.step()
            train_loss += _loss_val.detach().item()

        train_loss /= len(dataloader_train.dataset)
        train_losses.append(train_loss)

        if verbose > 0 and epoch % verbose == 0:
            print(f'Epoch: {epoch} | Training loss: {train_loss}')

    model.eval()
    return (model, train_losses)


def train_model_validate(model: 'SoftmaxMLP', dataset_train: 'PMDataset',
                         dataset_valid: 'PMDataset', epochs: int = 100,
                         batch_size: int = 1, patience: int = 8, verbose: int = 0,
                         **kwargs) -> Tuple['SoftmaxMLP', List[float], List[float]]:

    dataloader_train = DataLoader(dataset_train, batch_size=batch_size, shuffle=True)
    dataloader_valid = DataLoader(dataset_valid, batch_size=len(dataset_valid), shuffle=True)

    opt = torch.optim.Adam(model.parameters(), **kwargs)
    loss = nn.BCELoss()

    train_losses = []
    valid_losses = []

    model.train()

    for epoch in range(epochs):

        train_loss = 0.0
        for batch in dataloader_train:

            opt.zero_grad()
            pred = model(batch['X'])
            target = batch['y']
            _loss_val = loss(pred, target)
            _loss_val.backward()
            opt.step()
            train_loss += _loss_val.detach().item()

        train_loss /= len(dataset_train)
        train_losses.append(train_loss)

        valid_loss = 0.0
        for batch in dataloader_valid:

            v_pred = model(batch['X'])
            v_target = batch['y']
            v_loss = loss(v_pred, v_target)
            valid_loss += v_loss.detach().item()

        valid_loss /= len(dataset_valid)
        valid_losses.append(valid_loss)

        if verbose > 0 and epoch % verbose == 0:
            print(f'Epoch: {epoch} | Training loss: {train_loss} | Validation loss: {valid_loss}')

        if epoch == 0:
            best_valid_loss = valid_loss
            best_model_state = model.state_dict()
            _epoch_since_best = 0
        elif valid_loss < best_valid_loss:
            best_valid_loss = valid_loss
            best_model_state = model.state_dict()
            _epoch_since_best = 0
        else:
            _epoch_since_best += 1
            if _epoch_since_best > patience:
                model.load_state_dict(best_model_state)
                break

    model.eval()
    return (model, train_losses, valid_losses)
