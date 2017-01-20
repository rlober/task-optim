#task-compatibility-optimization

## Generate python documentation:

```
cd [path_to_repo]/optimization-scripts/docs
sphinx-apidoc -f -o _source/ ../task_optim/
make html
```

# best reaching params

```
bo_solver_parameters  = {'max_iter':maxIter, 'tolfun':tolerance, 'par':1000.0, 'kernel':'Matern52', 'acquisition':'EI', 'maximizer':'Direct'}
cma_solver_parameters = {'max_iter':maxIter, 'tolfun':tolerance, 'initial_sigma':0.1}
```

# best standing params

```
bo_solver_parameters  = {'max_iter':maxIter, 'tolfun':tolerance, 'par':10.0, 'kernel':'Matern52', 'acquisition':'LCB', 'maximizer':'Direct'}
cma_solver_parameters = {'max_iter':maxIter, 'tolfun':tolerance, 'initial_sigma':0.1}
```
