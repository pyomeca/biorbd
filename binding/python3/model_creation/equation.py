import ast


class Equation:
    def __init__(self, equation: str):
        """
        The right-hand side of the equation that defines the axis. Any word (that is, anything which is not numbers
        and math symbols) must refer to an actual marker of the model.

        Parameters
        ----------
        equation
            The right-hand side of the equation that defines the axis.
            Any word (that is, anything which is not numbers and math symbols) must refer to a marker of the model.
            The allowed math symbols are: +-*/()
        """

        self.equation = equation

    @property
    def marker_names(self) -> tuple[str, ...]:
        return tuple(
            node.id for node in ast.walk(ast.parse(self.equation))
            if isinstance(node, ast.Name)
        )

    def eval(self, markers: dict) -> np.ndarray:
        """
        Evaluate the equation with provided marker positions

        Parameters
        ----------
        markers
            A dictionary of the marker positions. All markers must appear in the dictionary

        Returns
        -------
        The evaluated equation
        """

        # Replace the values in the equation to evaluate
        eq = self.equation
        for name in self.marker_names:
            if name not in markers:
                raise RuntimeError(f"The marker {name} was not found in the provided directory")
            eq = eq.replace(name, markers[name])

        return self.__safe_eval(self.equation)

    @staticmethod
    def __safe_eval(s):

        def checkmath(x, *args):
            if x not in [x for x in dir(math) if not "__" in x]:
                raise SyntaxError(f"Unknown func {x}()")
            fun = getattr(math, x)
            return fun(*args)

        binOps = {
            ast.Add: operator.add,
            ast.Sub: operator.sub,
            ast.Mult: operator.mul,
            ast.Div: operator.truediv,
            ast.Mod: operator.mod,
            ast.Pow: operator.pow,
            ast.Call: checkmath,
            ast.BinOp: ast.BinOp,
        }

        unOps = {
            ast.USub: operator.neg,
            ast.UAdd: operator.pos,
            ast.UnaryOp: ast.UnaryOp,
        }

        ops = tuple(binOps) + tuple(unOps)

        tree = ast.parse(s, mode='eval')

        def _eval(node):
            if isinstance(node, ast.Expression):
                logger.debug("Expr")
                return _eval(node.body)
            elif isinstance(node, ast.Str):
                logger.debug("Str")
                return node.s
            elif isinstance(node, ast.Num):
                logger.debug("Num")
                return node.value
            elif isinstance(node, ast.Constant):
                logger.info("Const")
                return node.value
            elif isinstance(node, ast.BinOp):
                logger.debug("BinOp")
                if isinstance(node.left, ops):
                    left = _eval(node.left)
                else:
                    left = node.left.value
                if isinstance(node.right, ops):
                    right = _eval(node.right)
                else:
                    right = node.right.value
                return binOps[type(node.op)](left, right)
            elif isinstance(node, ast.UnaryOp):
                logger.debug("UpOp")
                if isinstance(node.operand, ops):
                    operand = _eval(node.operand)
                else:
                    operand = node.operand.value
                return unOps[type(node.op)](operand)
            elif isinstance(node, ast.Call):
                args = [_eval(x) for x in node.args]
                r = checkmath(node.func.id, *args)
                return r
            else:
                raise SyntaxError(f"Bad syntax, {type(node)}")

        return _eval(tree)