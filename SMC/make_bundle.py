# =============================================================================
# make_bundle.py - Junta main_rs.py y todos sus modulos locales en UN solo archivo
# para subir al hub (main_bundle.py). Evita el lio de subir 6 archivos por BLE.
#
#   python SMC/make_bundle.py            (desde cualquier carpeta)
#   pybricksdev run ble main_bundle.py   (o abrir main_bundle.py en VS Code y F5)
#
# Hay que volver a correrlo cada vez que se edite algun .py del hub.
# En el bundle todos los modulos comparten UN namespace: usar `from modulo import nombre`
# (nunca `import modulo` + modulo.nombre) y no repetir nombres globales entre modulos.
# No modifica ninguno de los originales. Quita de cada modulo: imports locales,
# imports repetidos (se hoistean arriba), bloques `if __name__ == "__main__"` (salvo
# el de main_rs) y las funciones run_test (codigo de prueba, ahorra RAM en el hub).
# =============================================================================
import ast, os, sys, time

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
# Orden de dependencias: cada modulo despues de los que importa.
ORDER = ['vehicleConstants', 'controlConstants', 'reeds_shepp', 'dubinsPathPlanning',
         'smc_control', 'settings', 'velocity_profile', 'reference', 'odometry',
         'hardware', 'comms', 'trajectory', 'controllers', 'main_rs']
LOCAL = set(ORDER)


def strip_comments_and_docstrings(text):
    """Reduce el tamano del bundle (el hub compila el archivo en RAM): quita los
    comentarios de linea completa, los comentarios al final de linea y las docstrings.
    Los .py originales NO se tocan. Usa tokenize/ast, no regex: no rompe strings."""
    import io, tokenize
    # docstrings: lineas [lineno, end_lineno] de la primera Expr-string de cada cuerpo
    doc_ranges = []
    tree = ast.parse(text)
    for node in ast.walk(tree):
        if isinstance(node, (ast.Module, ast.FunctionDef, ast.ClassDef)):
            body = node.body
            if body and isinstance(body[0], ast.Expr) and isinstance(getattr(body[0], 'value', None), ast.Constant) \
                    and isinstance(body[0].value.value, str):
                doc_ranges.append((body[0].lineno, body[0].end_lineno, len(body) == 1,
                                   body[0].col_offset))
    lines = text.split('\n')
    for a, b, only, col in sorted(doc_ranges, reverse=True):
        if only:
            lines[a - 1:b] = [' ' * col + 'pass']          # cuerpo vacio -> pass
        else:
            del lines[a - 1:b]
    text = '\n'.join(lines)
    # comentarios
    out = []
    toks = list(tokenize.generate_tokens(io.StringIO(text).readline))
    cut = {}   # (fila) -> columna desde la que se corta (comentario)
    for t in toks:
        if t.type == tokenize.COMMENT:
            cut[t.start[0]] = t.start[1]
    for i, l in enumerate(text.split('\n'), 1):
        if i in cut:
            l = l[:cut[i]].rstrip()
        if l.strip() == '':
            continue
        out.append(l)
    return '\n'.join(out)


def read(name):
    with open(os.path.join(REPO, name + '.py'), encoding='utf-8') as f:
        return f.read()


def build():
    hoisted = []          # imports no locales, sin repetir, en orden de aparicion
    bodies = []
    for mod in ORDER:
        src = read(mod)
        lines = src.split('\n')
        drop = set()
        for node in ast.parse(src).body:
            rng = range(node.lineno - 1, node.end_lineno)
            if isinstance(node, (ast.Import, ast.ImportFrom)):
                names = [node.module] if isinstance(node, ast.ImportFrom) else [a.name for a in node.names]
                if not any(n.split('.')[0] in LOCAL for n in names):
                    seg = '\n'.join(lines[node.lineno - 1:node.end_lineno]).strip()
                    if seg not in hoisted:
                        hoisted.append(seg)
                drop.update(rng)
            elif (isinstance(node, ast.If) and isinstance(node.test, ast.Compare)
                  and getattr(node.test.left, 'id', '') == '__name__' and mod != 'main_rs'):
                drop.update(rng)
            elif isinstance(node, ast.FunctionDef) and node.name == 'run_test' and mod != 'main_rs':
                # incluye decoradores/comentarios pegados: solo el cuerpo de la funcion
                drop.update(rng)
        kept = [l for i, l in enumerate(lines) if i not in drop]
        text = strip_comments_and_docstrings('\n'.join(kept).strip('\n'))
        bodies.append('# ' + '=' * 76 + '\n# ---- modulo: {}.py\n# '.format(mod) + '=' * 76 + '\n' + text)
    head = ('# ' + '=' * 76 + '\n'
            '# main_bundle.py - GENERADO por SMC/make_bundle.py ({}). NO EDITAR A MANO:\n'
            '# editar los .py originales y volver a correr el script.\n'
            '# ' + '=' * 76 + '\n').format(time.strftime('%Y-%m-%d %H:%M'))
    return head + '\n'.join(hoisted) + '\n\n\n' + '\n\n\n'.join(bodies) + '\n'


if __name__ == '__main__':
    out = build()
    compile(out, 'main_bundle.py', 'exec')            # falla si quedo mal
    dest = os.path.join(REPO, 'main_bundle.py')
    with open(dest, 'w', encoding='utf-8', newline='\n') as f:
        f.write(out)
    print('OK -> {} ({} lineas, {} bytes)'.format(dest, out.count('\n'), len(out.encode('utf-8'))))
