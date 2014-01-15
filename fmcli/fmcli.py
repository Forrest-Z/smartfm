import json
import tmuxp
import urwid

class TmuxManager(object):
    def __init__(self):
        self.server = tmuxp.Server()
        sessions = self.server.attached_sessions()
        assert len(sessions) == 1
        self.session = tmuxp.Session(server=self.server, session_id=sessions[0]['session_id'])
        self.window = self.session.attached_window()

        self.window.tmux('set-window-option', 'main-pane-width', '50')
        self.window.tmux('set-window-option', 'remain-on-exit', 'on')
        self.window.tmux('set-window-option', 'set-remain-on-exit', 'on')

    def layout(self):
        self.window.tmux('select-layout', 'main-vertical')

    def split_window(self, command, target=None, attach=True):
        """Split window and return the created :class:`Pane`.

        .. note::

            :term:`tmux(1)` will move window to the new pane if the
            ``split-window`` target is off screen. tmux handles the ``-d`` the
            same way as ``new-window`` and ``attach`` in
            :class:`Session.new_window`.

            By default, this will make the window the pane is created in
            active. To remain on the same window and split the pane in another
            target window, pass in ``attach=False``.


        Used for splitting window and holding in a python object.

        :param attach: make new window the current window after creating it,
                       default True.
        :type attach: bool
        :param target: ``target_pane`` to split.
        :type target: bool

        :rtype: :class:`Pane`

        """

        pformats = ['session_name', 'session_id',
                    'window_index', 'window_id'] + tmuxp.formats.PANE_FORMATS
        tmux_formats = ['#{%s}\t' % f for f in pformats]

        #'-t%s' % self.attached_pane().get('pane_id'),
        # 2013-10-18 LOOK AT THIS, rm'd it..
        tmux_args = tuple()

        if target:
            tmux_args += ('-t%s' % target,)
        #else:
            #tmux_args += ('-t%s' % self.panes[0].get('pane_id'),)

        tmux_args += (
            '-P', '-F%s' % ''.join(tmux_formats)     # output
        )

        if not attach:
            tmux_args += ('-d',)

        tmux_args += (command,)

        pane = self.window.tmux(
            'split-window',
            *tmux_args
        )

        # tmux < 1.7. This is added in 1.7.
        if pane.stderr:
            raise tmuxp.exc.TmuxpException(pane.stderr)
            if 'pane too small' in pane.stderr:
                pass

            raise tmuxp.exc.TmuxpException(pane.stderr, self._TMUX, self.panes)
        else:
            pane = pane.stdout[0]

            pane = dict(zip(pformats, pane.split('\t')))

            # clear up empty dict
            pane = dict((k, v) for k, v in pane.items() if v)

        return tmuxp.Pane(window=self.window, **pane)

    def new_pane(self, command):
        #pane = self.window.tmux('split-window', '-P', '-d', command)
        pane = self.split_window(command, attach=False)
        self.layout()
        return pane

    def kill_pane(self, pane):
        self.window.tmux('kill-pane', '-t', pane['pane_id'])
        return

tmuxman = TmuxManager()

class Task(object):
    def __init__(self, name, command, tmux):
        self.name = name
        self.command = command
        self.tmux = tmux
        self.pane = None
        self.status = 'STOPPED'

    def spawn(self):
        self.pane = self.tmux.new_pane(self.command)
        self.status = 'OK'

    def start(self):
        if self.status == 'STOPPED':
            self.spawn()

    def terminate(self):
        if self.status == 'OK':
            self.tmux.kill_pane(self.pane)
            self.pane = None
            self.status = 'STOPPED'

STATUS_COLORS = {'OK': 'light green', 'STOPPED': 'dark red'}

class TaskButton(urwid.Columns):
    def __init__(self, task):
        self.task = task
        self.button = urwid.Button(task.name, self.on_task_clicked)
        self.status = urwid.Text(task.status, align='right')
        self.__super.__init__([self.button, self.status])
        self.update_status()
        self._selectable = True

    def keypress(self, size, key):
        if key == 's':
            self.task.start()
            self.update_status()
        elif key == 't':
            self.task.terminate()
            self.update_status()
        elif key == 'r':
            self.task.terminate()
            self.task.start()
            self.update_status()
        else:
            return self.__super.keypress(size, key)


    def on_task_clicked(self, button):
        self.task.spawn()
        self.update_status()

    def update_status(self):
        status = self.task.status
        attr = 'status_' + status.lower()
        self.status.set_text((attr, status))


urwid.command_map['j'] = 'cursor down'
urwid.command_map['k'] = 'cursor up'

def build_tasks():
    config = json.load(open('config.json'))
    tasks = config['tasks']
    ts = [Task(name, command, tmuxman) for name, command in tasks.items()]
    return ts

def task_menu(tasks):
    body = []
    for t in tasks:
        #button = urwid.Button(t.name)
        #urwid.connect_signal(button, 'click', on_task_clicked, t)
        button = TaskButton(t)
        body.append(urwid.AttrMap(button, None, focus_map='reversed'))
    lb = urwid.ListBox(urwid.SimpleFocusListWalker(body))
    return lb

DOC = """
s: start
t: terminate
r: restart
q: quit
"""

PALETTE = [
        ('reversed', 'standout', ''),
        ('status_ok', 'light green', ''),
        ('status_stopped', 'dark red', ''),
            ]

def main():
    tasks = build_tasks()

    def exit_on_q(key):
        if key in ('q', 'Q'):
            for t in tasks:
                t.terminate()
            raise urwid.ExitMainLoop()

    main = urwid.Padding(task_menu(tasks))
    frame = urwid.Frame(main, footer=urwid.Text(DOC))
    #top = urwid.Overlay(main, urwid.SolidFill(u'\N{MEDIUM SHADE}'),
        #align='center', width=('relative', 60),
        #valign='middle', height=('relative', 60),
        #min_width=20, min_height=9)
    loop = urwid.MainLoop(frame, palette=PALETTE, unhandled_input=exit_on_q)
    loop.run()

def test():
    import time
    t = TmuxManager()
    pane = t.new_pane('./test.sh')
    print pane.keys()
    print pane['pane_id']
    time.sleep(1)
    t.kill_pane(pane)

if __name__=='__main__':
    #test()
    main()
