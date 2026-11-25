"""Gera os layouts do AdvantageScope e do Elastic do robô atual (FRC_9460_Pendulo):
base swerve MAXSwerve (NEO + NEO 550) com Pigeon 2, sem Limelight.

Uso (na pasta do projeto):  python config/gerar_layouts.py

Saídas:
  config/advantagescope/layouts/FRC9460-Pendulo.json  (AdvantageScope: robô, simulação e arquivo .wpilog)
  src/main/deploy/Elastic-Pendulo-Odometria.json      (Elastic; vai para o robô no deploy)
  config/elastic/Elastic-Pendulo-Odometria.json       (a mesma, para abrir no PC: File > Open Layout)

AdvantageScope: ao vivo (NetworkTables, sem AdvantageKit) ele chama as chaves de
"NT:/SmartDashboard/...", igual ao .wpilog do DataLogManager. Por isso um layout só, com "NT:".

As chaves são as que o código publica em SmartDashboard/Swerve e SmartDashboard/Robo
(Swerve.java, SwerveVisualizer.java e Robot.java). O script confere no código-fonte que
cada chave usada existe e para com erro se faltar alguma.
"""
from pathlib import Path
import json
import re
import uuid

ROOT = Path(__file__).resolve().parents[1]

MODEL = 'FRC9460 - Base Swerve'  # "name" do config.json do modelo 3D (FRC Projeto 2027/config/advantagescope/assets)
FIELD = 'FRC:2026 Field'
MAX_SPEED = 4.8           # m/s, roda livre do MAXSwerve 14T (SwerveConstants.DRIVE_FREE_SPEED_MPS)
WHEEL_BASE = 0.5842       # m, 23" entre os centros das rodas nos dois sentidos

CYAN, ORANGE, GREEN, PURPLE, RED, BLUE, YELLOW = '#00bcd4', '#e48b32', '#3b875a', '#b47be2', '#e53935', '#2b66a2', '#f4d03f'
MODULES = [('FE', 'frente esquerda', CYAN), ('FD', 'frente direita', ORANGE),
           ('TE', 'trás esquerda', GREEN), ('TD', 'trás direita', PURPLE)]

USED = set()  # chaves (sem prefixo) usadas nos layouts, conferidas no código no fim


def k(prefix, key):
    USED.add(key)
    return prefix + key

# ---------------------------------------------------------------- AdvantageScope


def _uuid(title):
    return uuid.uuid5(uuid.NAMESPACE_URL, 'frc9460-pendulo/' + title).hex


def obj(kind, key, log_type, **options):
    return dict(type=kind, logKey=key, logType=log_type, visible=True, options=options)


def line(key, color, kind='smooth', log_type='Number'):
    return dict(type=kind, logKey=key, logType=log_type, visible=True, options=dict(color=color, size='normal'))


def stripes(key, color, log_type='Boolean'):
    return dict(type='stripes', logKey=key, logType=log_type, visible=True, options=dict(color=color))


def graph(title, left=(), right=(), discrete=()):
    controller = dict(
        leftSources=list(left), rightSources=list(right), discreteSources=list(discrete),
        leftLockedRange=None, rightLockedRange=None,
        leftUnitConversion=dict(autoTarget=None, preset=None),
        rightUnitConversion=dict(autoTarget=None, preset=None),
        leftFilter=0, rightFilter=0)
    return dict(type=1, title=title, controller=controller, controllerUUID=_uuid(title),
                renderer=None, controlsHeight=230)


def field3d(title, sources, camera_index=-1, position=(0, 6, -12), target=(0, 0.3, 0)):
    return dict(type=3, title=title, controller=dict(sources=sources, game=FIELD),
                controllerUUID=_uuid(title),
                renderer=dict(cameraIndex=camera_index, orbitFov=50,
                              cameraPosition=list(position), cameraTarget=list(target)),
                controlsHeight=260)


def field2d(title, sources):
    return dict(type=2, title=title, controller=dict(sources=sources, field=FIELD, orientation=0, size='large'),
                controllerUUID=_uuid(title), renderer=None, controlsHeight=200)


def table(title, keys):
    return dict(type=4, title=title, controller=list(keys), controllerUUID=_uuid(title),
                renderer=None, controlsHeight=0)


def advantagescope(p='NT:/SmartDashboard/'):
    """p = prefixo das chaves no AdvantageScope (ao vivo e no .wpilog: 'NT:/SmartDashboard/')."""
    S = lambda key: k(p, 'Swerve/' + key)
    robot = obj('robot', S('Pose'), 'Pose2d', model=MODEL)
    # Fontes "filhas" do robô: precisam vir logo depois dele na lista.
    wheels = [obj('swerveStates', S('EstadosMedidos'), 'SwerveModuleState[]', color='#00ffff', arrangement='0,1,2,3'),
              obj('swerveStates', S('EstadosAlvo'), 'SwerveModuleState[]', color='#ff8c00', arrangement='0,1,2,3')]
    trail = obj('trajectory', S('Rastro'), 'Pose2d[]', color=YELLOW, size='normal')

    def pair_turn(a, b):
        return graph(f'Giro {a[0]} e {b[0]}: alvo (degrau) x medido (graus)',
                     [line(S(f'{m}/AnguloGraus'), c) for m, _, c in (a, b)]
                     + [line(S(f'{m}/AlvoAnguloGraus'), c, 'stepped') for m, _, c in (a, b)],
                     [], [stripes(S('Estado'), BLUE, 'String')])

    def pair_drive(a, b):
        return graph(f'Tração {a[0]} e {b[0]}: alvo (degrau) x medido (m/s)',
                     [line(S(f'{m}/VelocidadeMetrosPorSeg'), c) for m, _, c in (a, b)]
                     + [line(S(f'{m}/AlvoVelocidadeMetrosPorSeg'), c, 'stepped') for m, _, c in (a, b)],
                     [], [stripes(S('Estado'), BLUE, 'String')])

    tabs = [
        field3d('1 - Campo 3D: robô, rodas e rastro', [robot, *wheels, trail]),
        field3d('2 - Robô de perto: rodas (ciano = medido, laranja = alvo)', [robot, *wheels],
                camera_index=-2, position=(1.4, 1.2, 1.4)),
        field2d('3 - Campo 2D: odometria e rastro', [obj('robot', S('Pose'), 'Pose2d', bumpers=''), *wheels, trail]),
        dict(type=9, title='4 - Swerve: alvo (laranja) x medido (ciano)', controller=dict(
            sources=[obj('states', S('EstadosMedidos'), 'SwerveModuleState[]', color='#00ffff', arrangement='0,1,2,3'),
                     obj('states', S('EstadosAlvo'), 'SwerveModuleState[]', color='#ff8c00', arrangement='0,1,2,3'),
                     obj('chassisSpeeds', S('VelocidadesChassi'), 'ChassisSpeeds', color=PURPLE),
                     obj('rotation', S('Rotacao'), 'Rotation2d')],
            maxSpeed=MAX_SPEED, sizeX=WHEEL_BASE, sizeY=WHEEL_BASE, orientation=1),
            controllerUUID=_uuid('swerve'), renderer=None, controlsHeight=200),
        graph('5 - Chassi: velocidade (m/s) | giro (graus/s)',
              [line(S('VelocidadeMetrosPorSeg'), CYAN)], [line(S('GiroGrausPorSeg'), PURPLE)],
              [stripes(S('Pilotagem'), BLUE, 'String'), stripes(S('Estado'), GREEN, 'String'),
               stripes(S('Comando'), ORANGE, 'String')]),
        graph('6 - Odometria: X e Y (m) | frente do robô (graus)',
              [line(S('PoseXMetros'), RED), line(S('PoseYMetros'), GREEN)],
              [line(S('FrenteGraus'), BLUE), line(S('Giroscopio/YawGraus'), YELLOW, 'stepped')],
              [stripes(S('Giroscopio/Conectado'), GREEN), stripes(S('Giroscopio/Estado'), BLUE, 'String')]),
        pair_turn(MODULES[0], MODULES[1]),
        pair_turn(MODULES[2], MODULES[3]),
        pair_drive(MODULES[0], MODULES[1]),
        pair_drive(MODULES[2], MODULES[3]),
        graph('11 - Corrente: tração | giro (A)',
              [line(S(f'{m}/Tracao/CorrenteAmps'), c) for m, _, c in MODULES],
              [line(S(f'{m}/Giro/CorrenteAmps'), c, 'stepped') for m, _, c in MODULES]),
        graph('12 - Tensão aplicada: tração | giro (V)',
              [line(S(f'{m}/Tracao/Tensao'), c) for m, _, c in MODULES],
              [line(S(f'{m}/Giro/Tensao'), c, 'stepped') for m, _, c in MODULES]),
        graph('13 - Encoders absolutos: leitura | diferença para o NEO 550 (graus)',
              [line(S(f'{m}/EncoderAbsolutoGraus'), c) for m, _, c in MODULES],
              [line(S(f'{m}/DiferencaAbsolutoGraus'), c, 'stepped') for m, _, c in MODULES],
              [*[stripes(S(f'{m}/EncoderAbsolutoConectado'), c) for m, _, c in MODULES],
               stripes(S('EncodersAbsolutosRepetidos'), RED, 'String')]),
        graph('14 - Saúde: bateria (V)',
              [line(k(p, 'Robo/BateriaVolts'), GREEN)], [],
              [stripes(k(p, 'Robo/Brownout'), RED), *[stripes(S(f'{m}/Conectado'), c) for m, _, c in MODULES],
               stripes(S('Falha'), RED, 'String')]),
        table('15 - Status e calibração dos módulos',
              [S('Pilotagem'), S('Estado'), S('Falha'), S('Giroscopio/Estado')]
              + [S(f'{m}/{x}') for m, _, _ in MODULES
                 for x in ('Configuracao', 'FalhasAtivas', 'OffsetParaCopiar', 'Giro/RazaoAbsolutoPorMotor',
                           'EncoderAbsolutoHz', 'ReiniciosTracao', 'Tracao/Firmware', 'Giro/Firmware')]),
        dict(type=8, title='16 - Controle do piloto', controller=['Xbox Controller (Blue)'] + ['None'] * 5,
             controllerUUID=_uuid('joysticks'), renderer=None, controlsHeight=85),
    ]
    for i, t in enumerate(tabs[6:14], start=7):  # numera os gráficos dos módulos
        if not t['title'][0].isdigit():
            t['title'] = f'{i} - ' + t['title']
    expanded = ['NT:', 'NT:/SmartDashboard', 'NT:/SmartDashboard/Swerve', '/SmartDashboard', '/SmartDashboard/Swerve']
    return dict(hubs=[dict(x=-8, y=-8, width=1552, height=832, state=dict(
        sidebar=dict(width=300, expanded=expanded),
        tabs=dict(selected=0, tabs=tabs)))], satellites=[], version='26.0.0')

# ---------------------------------------------------------------- Elastic


NT = '/SmartDashboard/'
GRID = 96
GREEN_ARGB, RED_ARGB = 4283215696, 4294198070
ARGB = {CYAN: 0xFF00BCD4, ORANGE: 0xFFE48B32, GREEN: 0xFF3B875A, PURPLE: 0xFFB47BE2, RED: 0xFFE53935,
        BLUE: 0xFF2B66A2, YELLOW: 0xFFF4D03F}


def w(title, key, x, y, cols, rows, kind='Text Display', data_type='string', **extra):
    props = dict(topic=k(NT, key), period=0.06)
    if data_type:
        props['data_type'] = data_type
    if kind == 'Text Display':
        props['show_submit_button'] = False
    if kind == 'Boolean Box':
        props.update(true_color=GREEN_ARGB, false_color=RED_ARGB, true_icon='None', false_icon='None')
    props.update(extra)
    return dict(title=title, x=float(x * GRID), y=float(y * GRID), width=float(cols * GRID),
                height=float(rows * GRID), type=kind, properties=props)


def ok(title, key, x, y, cols=2, inverted=False):
    widget = w(title, key, x, y, cols, 1, 'Boolean Box', 'boolean')
    if inverted:  # verde quando FALSO (ex.: brownout)
        widget['properties'].update(true_color=RED_ARGB, false_color=GREEN_ARGB)
    return widget


def num(title, key, x, y, cols=2):
    return w(title, key, x, y, cols, 1, data_type='double')


def chart(title, key, x, y, cols, rows, color, lo=None, hi=None, seconds=10.0):
    extra = dict(time_displayed=seconds, color=ARGB[color], line_width=2.0)
    if lo is not None:
        extra.update(min_value=lo, max_value=hi)
    return w(title, key, x, y, cols, rows, 'Graph', 'double', **extra)


def command(title, key, x, y, cols=2):
    return dict(title=title, x=float(x * GRID), y=float(y * GRID), width=float(cols * GRID), height=float(GRID),
                type='Command', properties=dict(topic=k(NT, key), period=0.06, show_type=False))


odometria = [
    w('Campo: odometria (amarelo = rastro dos últimos 15 s)', 'Swerve/Campo', 0, 0, 7, 4, 'Field', None,
      field_game='Rebuilt', robot_width=0.85, robot_length=0.85, show_other_objects=True, show_trajectories=True,
      field_rotation=0.0, robot_color=4278238420, trajectory_color=ARGB[YELLOW], show_robot_outside_widget=True),
    w('Rodas (medido)', 'Swerve/Modulos', 7, 0, 3, 3, 'SwerveDrive', None,
      show_robot_rotation=True, rotation_unit='Radians'),
    w('Pilotagem (Y)', 'Swerve/Pilotagem', 10, 0, 2, 1),
    num('X (m)', 'Swerve/PoseXMetros', 10, 1),
    num('Y (m)', 'Swerve/PoseYMetros', 10, 2),
    num('Frente (graus)', 'Swerve/FrenteGraus', 10, 3),
    num('Velocidade (m/s)', 'Swerve/VelocidadeMetrosPorSeg', 7, 3),
    num('Giro (graus/s)', 'Swerve/GiroGrausPorSeg', 9, 3, 1),
    ok('Pigeon', 'Swerve/Giroscopio/Conectado', 0, 4),
    w('Estado do swerve', 'Swerve/Estado', 2, 4, 2, 1),
    w('Comando', 'Swerve/Comando', 4, 4, 2, 1),
    w('Velocidade (BACK)', 'Swerve/Velocidade', 6, 4, 2, 1),
    num('Bateria (V)', 'Robo/BateriaVolts', 8, 4),
    command('Zerar frente', 'Swerve/ZerarFrente', 10, 4),
    w('Falha (vazio = NENHUMA)', 'Swerve/Falha', 0, 5, 12, 1),
    chart('Velocidade (m/s)', 'Swerve/VelocidadeMetrosPorSeg', 0, 6, 4, 2, CYAN, 0.0, MAX_SPEED),
    chart('Frente do robô (graus)', 'Swerve/FrenteGraus', 4, 6, 4, 2, BLUE, -180.0, 180.0),
    chart('Giro do robô (graus/s)', 'Swerve/GiroGrausPorSeg', 8, 6, 4, 2, PURPLE, -400.0, 400.0),
]

modulos = []
for i, (m, label, color) in enumerate(MODULES):
    x = i * 3
    modulos += [
        w(f'{m} ({label})', f'Swerve/{m}/Configuracao', x, 0, 3, 1),
        ok('OK', f'Swerve/{m}/Conectado', x, 1, 1),
        ok('Absoluto', f'Swerve/{m}/EncoderAbsolutoConectado', x + 1, 1, 2),
        chart(f'{m}: ângulo medido (graus)', f'Swerve/{m}/AnguloGraus', x, 2, 3, 2, color, -180.0, 180.0),
        num('Alvo (graus)', f'Swerve/{m}/AlvoAnguloGraus', x, 4, 3),
        chart(f'{m}: velocidade (m/s)', f'Swerve/{m}/VelocidadeMetrosPorSeg', x, 5, 3, 2, color, -MAX_SPEED, MAX_SPEED),
        num('Tração (A)', f'Swerve/{m}/Tracao/CorrenteAmps', x, 7, 1),
        num('Giro (A)', f'Swerve/{m}/Giro/CorrenteAmps', x + 1, 7, 1),
        num('Alvo (m/s)', f'Swerve/{m}/AlvoVelocidadeMetrosPorSeg', x + 2, 7, 1),
    ]

calibracao = [w('Encoders repetidos (2 DIOs no mesmo sinal)', 'Swerve/EncodersAbsolutosRepetidos', 0, 0, 12, 1)]
for i, (m, label, color) in enumerate(MODULES):
    y = 1 + i * 2
    calibracao += [
        ok(f'{m} absoluto', f'Swerve/{m}/EncoderAbsolutoConectado', 0, y, 2),
        num('Leitura (graus)', f'Swerve/{m}/EncoderAbsolutoGraus', 2, y),
        num('Ângulo (graus)', f'Swerve/{m}/AnguloGraus', 4, y),
        num('Absoluto − NEO 550', f'Swerve/{m}/DiferencaAbsolutoGraus', 6, y),
        num('Razão abs/NEO', f'Swerve/{m}/Giro/RazaoAbsolutoPorMotor', 8, y),
        num('Hz (~976)', f'Swerve/{m}/EncoderAbsolutoHz', 10, y),
        w(f'{m}: linha para SwerveConstants (rodas retas, desabilitado)', f'Swerve/{m}/OffsetParaCopiar', 0, y + 1, 12, 1),
    ]

saude = [
    w('Falha', 'Swerve/Falha', 0, 0, 12, 1),
    num('Bateria (V)', 'Robo/BateriaVolts', 0, 1),
    ok('Brownout', 'Robo/Brownout', 2, 1, inverted=True),
    ok('Pigeon', 'Swerve/Giroscopio/Conectado', 4, 1),
    w('Pigeon: estado', 'Swerve/Giroscopio/Estado', 6, 1, 6, 1),
    chart('Bateria (V)', 'Robo/BateriaVolts', 0, 2, 12, 2, GREEN, 6.0, 14.0, 30.0),
]
for i, (m, label, color) in enumerate(MODULES):
    x = i * 3
    saude += [
        w(f'{m}: avisos', f'Swerve/{m}/FalhasAtivas', x, 4, 3, 1),
        w('Tração fw', f'Swerve/{m}/Tracao/Firmware', x, 5, 3, 1),
        w('Giro fw', f'Swerve/{m}/Giro/Firmware', x, 6, 3, 1),
        num('Reinícios tração', f'Swerve/{m}/ReiniciosTracao', x, 7, 3),
    ]


def tab(name, widgets):
    return dict(name=name, grid_layout=dict(layouts=[], containers=widgets))


elastic = dict(version=1.0, grid_size=GRID, tabs=[
    tab('Odometria', odometria), tab('Módulos', modulos), tab('Calibração', calibracao), tab('Saúde', saude)])

# ---------------------------------------------------------------- Conferência e gravação


def published_keys():
    """Chaves que o código publica (SmartDashboard.put*, publishers e putData)."""
    keys = set()
    src = ROOT / 'src/main/java'
    for path in src.rglob('*.java'):
        text = path.read_text(encoding='utf-8')
        for m in re.finditer(r'SmartDashboard\.put\w+\(\s*(key\s*\+\s*)?"([^"]+)"', text):
            if m.group(1):
                for mod, _, _ in MODULES:
                    keys.add(f'Swerve/{mod}/' + m.group(2))
            else:
                keys.add(m.group(2))
        for m in re.finditer(r'get(?:Struct|StructArray)Topic\("([^"]+)"', text):
            keys.add('Swerve/' + m.group(1))
    return keys


def save(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
    return path.relative_to(ROOT)


if __name__ == '__main__':
    layout = advantagescope()
    missing = sorted(USED - published_keys())
    if missing:
        raise SystemExit('Chaves usadas nos layouts que o código não publica:\n  ' + '\n  '.join(missing))
    a = save(ROOT / 'config/advantagescope/layouts/FRC9460-Pendulo.json', layout)
    e = save(ROOT / 'src/main/deploy/Elastic-Pendulo-Odometria.json', elastic)
    save(ROOT / 'config/elastic/Elastic-Pendulo-Odometria.json', elastic)  # cópia para achar fácil no PC
    tabs = layout['hubs'][0]['state']['tabs']['tabs']
    print(f'AdvantageScope: {a} ({len(tabs)} abas)')
    print(f'Elastic: {e} ({len(elastic["tabs"])} abas)')
    print(f'{len(USED)} chaves conferidas no código')
