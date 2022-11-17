# Develop hooks

For having a stable development environment a [pre-commit hooks](https://github.com/pre-commit/pre-commit) are introduced to ensure
a coding style is production ready.


## How to use

To automatically check the code, please install `pre-commit` Python module with `pip install pre-commit`.

Once `pre-commit` is installed, install `pre-commit hooks` by `pre-commit install`.

Every time you perform any GIT operations a check is made and terminal output will inform the user about the warning and errors.

If it is possible `pre-commit` is trying to fix it. Add files and commit again. If this is not working a user input is required
in order to fix the errors.


## Hooks

- `autoflake`
- `isort`
- `black`
- `flake8`
- `yamllint`
- `prettier-xacro`
- `prettier-package-xml`
- `sort-package-xml`
- `end-of-file-fixer`
- `trailing-whitespace`
