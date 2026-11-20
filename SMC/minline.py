# minline.py - Transformacion del bundle: menos lineas fisicas (la tabla de numeros de linea del .mpy
# cuesta bytes por cada linea). Semantica identica (se verifica con la telemetria del stub).
import ast
SIMPLE = (ast.Assign, ast.AugAssign, ast.Expr, ast.AnnAssign, ast.Pass, ast.Global,
          ast.Return, ast.Break, ast.Continue, ast.Raise)

def _simple_line(body):
    return '; '.join(ast.unparse(n) for n in body)

def _all_simple(body):
    return all(isinstance(n, SIMPLE) for n in body)

def _emit(body, ind, out):
    i = 0
    while i < len(body):
        n = body[i]
        if isinstance(n, SIMPLE):
            j = i
            while j < len(body) and isinstance(body[j], SIMPLE):
                j += 1
            out.append(' ' * ind + _simple_line(body[i:j]))
            i = j
            continue
        _compound(n, ind, out)
        i += 1

def _block(head, body, ind, out):
    if _all_simple(body):
        out.append(' ' * ind + head + ' ' + _simple_line(body))
    else:
        out.append(' ' * ind + head)
        _emit(body, ind + 4, out)

def _compound(n, ind, out):
    if isinstance(n, (ast.FunctionDef, ast.ClassDef)):
        clone = type(n)(**{f: getattr(n, f) for f in n._fields})
        clone.body = [ast.Pass()]
        head = ast.unparse(ast.fix_missing_locations(clone)).split('\n')
        # unparse pone decoradores en lineas previas: se respetan
        for h in head[:-1]:
            out.append(' ' * ind + h)
        last = head[-1]
        assert last.rstrip().endswith('pass')
        last = last.rstrip()[:-4].rstrip()        # quita el 'pass' del clon
        _block(last, n.body, ind, out)
    elif isinstance(n, ast.If):
        _block('if ' + ast.unparse(n.test) + ':', n.body, ind, out)
        orelse = n.orelse
        while len(orelse) == 1 and isinstance(orelse[0], ast.If):
            _block('elif ' + ast.unparse(orelse[0].test) + ':', orelse[0].body, ind, out)
            orelse = orelse[0].orelse
        if orelse:
            _block('else:', orelse, ind, out)
    elif isinstance(n, ast.While):
        _block('while ' + ast.unparse(n.test) + ':', n.body, ind, out)
    elif isinstance(n, ast.For):
        _block('for ' + ast.unparse(n.target) + ' in ' + ast.unparse(n.iter) + ':', n.body, ind, out)
    elif isinstance(n, ast.Try):
        assert not n.orelse and not n.finalbody, 'try/else/finally no soportado'
        _block('try:', n.body, ind, out)
        for h in n.handlers:
            _block('except' + (' ' + ast.unparse(h.type) if h.type else '') + ':', h.body, ind, out)
    else:
        assert not hasattr(n, 'body'), 'sentencia compuesta no soportada: ' + type(n).__name__
        out.append(' ' * ind + ast.unparse(n))

def minimize_lines(src):
    out = []
    _emit(ast.parse(src).body, 0, out)
    return '\n'.join(out) + '\n'
