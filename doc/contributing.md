# Contributing to `biorbd`
All contributions, bug reports, bug fixes, documentation improvements, enhancements, and ideas are welcome.
We recommend going through the list of [`issues`](https://github.com/pyomeca/biorbd/issues) to find issues that interest you, preferable those tagged with `good first issue`.
You can then get your development environment setup with the following instructions.

## Forking `biorbd

You will need your own fork to work on the code.
Go to the [biorbd project page](https://github.com/pyomeca/biorbd/) and hit the `Fork` button.
You will want to clone your fork to your machine:

```bash
git clone https://github.com/your-user-name/biorbd.git
```

## Creating and activating conda environment

Before starting any development, we recommend that you create an isolated development environment. 
The easiest and most efficient way (due to the numerous dependencies of `biorbd`) is to use an anaconda virtual environment and to create it based on the `environment_X.yml` file (where `X` is the desired backend, that is `eigen` or `casadi`). 

- Install [miniconda](https://conda.io/miniconda.html)
- `cd` to the biorbd source directory
- Install biorbd dependencies with:

```bash
conda env create -f environment_X.yml
conda activate biorbd_X
```
Where (again) `X` is the desired linear algebrea backend, that is `eigen` or `casadi`

## Implementing new features

Before starting to implement your new awesome feature, please discuss the implementation with the code owner to prevent any clashing with some other current developments. 
It is also a good idea to check the current opened pull-request to not redo something currently being developed. 
If your feature is mentioned in the issue section of GitHub, please assign it to yourself.
Otherwise, please open a new issue explaining what you are currently working on (and assign it to yourself!).

As soon as possible, you are asked to open a pull-request (see below) with a short but descriptive name. 
To tag that a pull-request is still in development, please add `[WIP]` at the beginning of the name.
Send commits that are as small as possible; 1 to 10 lines is probably a good guess, with again short but descriptive names. 
Be aware of the review done by the maintainers, they will contain useful tips and advices that should be integrated ASAP. 
Once you have responded to a specific comment, please respond `Done!` and tag it as resolved.

Make sure you create a unit test with numerical values for comparison.
As it may be tricky to create a numerical test for the CasADi's backend, you are welcome to ask for help as soon as you have created one for Eigen.  
Please note that if by accident you add a binary file in the history, your pull-request will be rejected and you will have to produce a new pull request free from the binary file. 

When you have completed the implementation of your new feature, navigate to your pull-request in GitHub and select `Pariterre` in the `Reviewers` drop menu. 
At the same time, if you think your review is ready to be merged, remove the `[WIP]` tag in the name (otherwise, your pull-request won't be merged). 
If your pull-request is accepted, there is nothing more to do. 
If changes are required, reply to all the comments and, as stated previously, respond `Done!` and tag it as resolved. 
Be aware that sometimes the maintainer can push modifications directly to your branch, so make sure to pull before continuing your work on the feature.

## Testing your code

Adding tests is required to get your development merged to the master branch. 
Therefore, it is worth getting in the habit of writing tests ahead of time so this is never an issue.
Each time you push to your pull-request, the `biorbd` test suite will run automatically.
However, we strongly encourage running the tests prior to submitting the pull-request.
If you configured your project using CMake and set `BUILD_TEST` to `ON`, the google test toolbox should download and compile itself.
You can thereafter run the test by running the binary `biorbd_tests` (the file will be at different place depending on the OS you are using).

## Convention of coding

`biorbd` tries to follow as much as possible a coherent standard, that is essentially:
  - camelCase;
  - 80 characters per line max;
  - the absence of `using namespace`
That said, if you are looking a the code, you will find unstandardize coding. 
That is the case because over the course of the past five years or so, the main coder ability has drastically changed, therefore historical non-compliant code remain, I appologize for that!
Therefore, as it is for now, unless you have good reasons to not follow a standardized approach, pull-requests are required to follow these recommendations. 

