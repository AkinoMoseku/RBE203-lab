using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.IO;
using System.IO.Ports;
using System.Text;
using System.Windows.Forms;

namespace RobotBluetoothJoystick
{
    // Tutorial protocol: [joystick,LH,LV,RH,RV]. Only LV is used.
    internal sealed class JoystickCommand
    {
        public const int DefaultLimit = 50;
        private int vertical;
        private int limit = DefaultLimit;
        private bool reverse;

        public int Vertical { get { return vertical; } }
        public int Limit { get { return limit; } }
        public bool Reverse { get { return reverse; } }
        public int Output
        {
            get
            {
                int value = (int)Math.Round(vertical * limit / 100.0,
                    MidpointRounding.AwayFromZero);
                return reverse ? -value : value;
            }
        }

        public void SetVertical(int value) { vertical = Clamp(value, -100, 100); }
        public void Center() { vertical = 0; }
        public void SetLimit(int value) { limit = Clamp(value, 0, 100); Center(); }
        public void SetReverse(bool value) { reverse = value; Center(); }
        public string Frame { get { return MakeFrame(Output); } }
        public static string MakeFrame(int value)
        {
            return "[joystick,0," +
                Clamp(value, -100, 100).ToString(System.Globalization.CultureInfo.InvariantCulture) +
                ",0,0]";
        }
        private static int Clamp(int value, int low, int high)
        {
            return Math.Max(low, Math.Min(high, value));
        }
    }

    internal sealed class VerticalJoystick : Control
    {
        private int vertical;
        private bool dragging;
        public event EventHandler VerticalChanged;
        public int Vertical { get { return vertical; } }

        public VerticalJoystick()
        {
            SetStyle(ControlStyles.UserPaint | ControlStyles.AllPaintingInWmPaint |
                ControlStyles.OptimizedDoubleBuffer | ControlStyles.ResizeRedraw |
                ControlStyles.Selectable, true);
            BackColor = Color.White;
            TabStop = true;
            AccessibleName = "左摇杆，上下拖动控制速度，松开回中";
            Cursor = Cursors.Hand;
        }

        public void ReleaseToCenter()
        {
            dragging = false;
            if (Capture) Capture = false;
            SetVertical(0);
        }

        private float Radius { get { return Math.Max(35, Math.Min(Width - 70, Height - 78) / 2f); } }
        private PointF CenterPoint { get { return new PointF(Width / 2f, Height / 2f); } }

        private void SetVertical(int value)
        {
            value = Math.Max(-100, Math.Min(100, value));
            if (vertical == value) return;
            vertical = value;
            Invalidate();
            EventHandler changed = VerticalChanged;
            if (changed != null) changed(this, EventArgs.Empty);
        }

        private void Track(Point point)
        {
            float travel = Math.Max(1, Radius - 30);
            SetVertical((int)Math.Round((CenterPoint.Y - point.Y) / travel * 100f));
        }

        protected override void OnMouseDown(MouseEventArgs e)
        {
            base.OnMouseDown(e);
            if (e.Button != MouseButtons.Left || !Enabled) return;
            Focus();
            dragging = true;
            Capture = true;
            Track(e.Location);
        }

        protected override void OnMouseMove(MouseEventArgs e)
        {
            base.OnMouseMove(e);
            if (dragging) Track(e.Location);
        }

        protected override void OnMouseUp(MouseEventArgs e)
        {
            base.OnMouseUp(e);
            if (e.Button == MouseButtons.Left) ReleaseToCenter();
        }

        protected override void OnMouseCaptureChanged(EventArgs e)
        {
            base.OnMouseCaptureChanged(e);
            if (!Capture)
            {
                dragging = false;
                SetVertical(0);
            }
        }

        protected override void OnLostFocus(EventArgs e)
        {
            base.OnLostFocus(e);
            ReleaseToCenter();
        }

        protected override void OnEnabledChanged(EventArgs e)
        {
            base.OnEnabledChanged(e);
            if (!Enabled) ReleaseToCenter();
            Invalidate();
        }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            Graphics g = e.Graphics;
            g.SmoothingMode = SmoothingMode.AntiAlias;
            PointF center = CenterPoint;
            float radius = Radius;
            RectangleF circle = new RectangleF(center.X - radius, center.Y - radius,
                radius * 2, radius * 2);
            Color accent = Enabled ? Color.FromArgb(0, 119, 145) : Color.FromArgb(151, 165, 176);
            using (SolidBrush fill = new SolidBrush(Color.FromArgb(243, 247, 250)))
                g.FillEllipse(fill, circle);
            using (Pen border = new Pen(Color.FromArgb(214, 225, 231), 2))
                g.DrawEllipse(border, circle);
            using (Pen guide = new Pen(Color.FromArgb(215, 226, 232), 2))
            {
                g.DrawLine(guide, center.X, center.Y - radius + 25, center.X, center.Y + radius - 25);
                g.DrawLine(guide, center.X - 25, center.Y, center.X + 25, center.Y);
            }
            float knobY = center.Y - vertical / 100f * (radius - 30);
            using (Pen shaft = new Pen(Color.FromArgb(105, 174, 188), 9))
            {
                shaft.StartCap = LineCap.Round;
                shaft.EndCap = LineCap.Round;
                g.DrawLine(shaft, center.X, center.Y, center.X, knobY);
            }
            using (SolidBrush knob = new SolidBrush(accent))
                g.FillEllipse(knob, center.X - 30, knobY - 30, 60, 60);
            TextRenderer.DrawText(g, "拖动", Font,
                Rectangle.Round(new RectangleF(center.X - 30, knobY - 12, 60, 25)),
                Color.White, TextFormatFlags.HorizontalCenter | TextFormatFlags.VerticalCenter);
            DrawCentered(g, "前推  /  LV +", new Rectangle(0, 5, Width, 25), accent);
            DrawCentered(g, "后拉  /  LV −", new Rectangle(0, Height - 30, Width, 25), accent);
        }

        private void DrawCentered(Graphics graphics, string text, Rectangle rectangle, Color color)
        {
            TextRenderer.DrawText(graphics, text, Font, rectangle, color,
                TextFormatFlags.HorizontalCenter | TextFormatFlags.VerticalCenter);
        }
    }

    internal sealed class JoystickForm : Form
    {
        private readonly JoystickCommand command = new JoystickCommand();
        private readonly Timer sendTimer = new Timer();
        private readonly object receiveLock = new object();
        private readonly StringBuilder pendingReceive = new StringBuilder();
        private readonly Queue<string> logLines = new Queue<string>();
        private volatile SerialPort port;
        private SerialPort failedPort;
        private string pendingError;
        private string receiveTail = "";
        private string lastLoggedFrame = "";
        private bool closing;
        private bool offscreenRendering;
        private ComboBox portBox;
        private Button connectButton;
        private Label statusLabel;
        private Label outputLabel;
        private Label frameLabel;
        private Label limitLabel;
        private TextBox logBox;
        private VerticalJoystick joystick;
        private TrackBar limitSlider;
        private CheckBox reverseBox;

        public JoystickForm()
        {
            Text = "平衡车 · 蓝牙左摇杆";
            ClientSize = new Size(1080, 780);
            MinimumSize = new Size(940, 740);
            StartPosition = FormStartPosition.CenterScreen;
            Font = new Font("Microsoft YaHei UI", 10f);
            BackColor = Color.FromArgb(243, 246, 249);
            ForeColor = Color.FromArgb(36, 51, 65);
            KeyPreview = true;
            AutoScaleMode = AutoScaleMode.Dpi;
            BuildInterface();
            RefreshPorts();
            sendTimer.Interval = 100; // A fresh command every 100 ms while connected.
            sendTimer.Tick += SendTimerTick;
            AddLog("尚未连接。请先在 Windows 中配对 HC-06（PIN：1234），再选择它的蓝牙 COM 端口。");
            AddLog("只使用左摇杆 LV；LH、RH、RV 始终为 0。回中表示速度目标归零，平衡控制仍可工作。");
            UpdateDisplay();
        }

        private static Label MakeLabel(string text, float size, bool bold)
        {
            return new Label {
                Text = text,
                Dock = DockStyle.Fill,
                Font = new Font("Microsoft YaHei UI", size, bold ? FontStyle.Bold : FontStyle.Regular),
                TextAlign = ContentAlignment.MiddleLeft,
                AutoEllipsis = true
            };
        }

        private static Button MakeButton(string text, int width)
        {
            return new Button {
                Text = text, Width = width, Height = 34,
                FlatStyle = FlatStyle.Flat, BackColor = Color.White,
                Margin = new Padding(0, 4, 10, 4)
            };
        }

        private void BuildInterface()
        {
            TableLayoutPanel root = new TableLayoutPanel {
                Dock = DockStyle.Fill, Padding = new Padding(24, 16, 24, 14),
                ColumnCount = 1, RowCount = 4
            };
            root.RowStyles.Add(new RowStyle(SizeType.Absolute, 68));
            root.RowStyles.Add(new RowStyle(SizeType.Absolute, 54));
            root.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            root.RowStyles.Add(new RowStyle(SizeType.Absolute, 38));
            Controls.Add(root);

            Panel heading = new Panel { Dock = DockStyle.Fill };
            Label title = MakeLabel("蓝牙左摇杆", 21, true);
            title.Dock = DockStyle.Top;
            title.Height = 37;
            Label subtitle = MakeLabel("HC-06  ·  9600 baud  ·  仅控制前后速度", 10, false);
            subtitle.Dock = DockStyle.Bottom;
            subtitle.Height = 25;
            subtitle.ForeColor = Color.FromArgb(88, 108, 124);
            heading.Controls.Add(title);
            heading.Controls.Add(subtitle);
            root.Controls.Add(heading, 0, 0);

            FlowLayoutPanel connection = new FlowLayoutPanel {
                Dock = DockStyle.Fill, WrapContents = false
            };
            portBox = new ComboBox {
                DropDownStyle = ComboBoxStyle.DropDownList, Width = 310,
                Margin = new Padding(0, 7, 10, 4), AccessibleName = "蓝牙串口"
            };
            Button refreshButton = MakeButton("刷新串口", 90);
            refreshButton.Click += delegate { if (port == null) RefreshPorts(); };
            connectButton = MakeButton("连接", 90);
            connectButton.BackColor = Color.FromArgb(0, 119, 145);
            connectButton.ForeColor = Color.White;
            connectButton.Click += delegate {
                if (port == null) Connect(); else Disconnect("连接已断开，速度目标已归零。", true);
            };
            Button centerButton = MakeButton("回中 · 空格", 125);
            centerButton.Click += delegate { CenterAndSend("手动回中"); };
            Button clearButton = MakeButton("清空日志", 90);
            clearButton.Click += delegate { logLines.Clear(); logBox.Clear(); };
            connection.Controls.Add(portBox);
            connection.Controls.Add(refreshButton);
            connection.Controls.Add(connectButton);
            connection.Controls.Add(centerButton);
            connection.Controls.Add(clearButton);
            root.Controls.Add(connection, 0, 1);

            TableLayoutPanel content = new TableLayoutPanel {
                Dock = DockStyle.Fill, ColumnCount = 2, RowCount = 1
            };
            content.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 43));
            content.ColumnStyles.Add(new ColumnStyle(SizeType.Percent, 57));
            GroupBox left = new GroupBox {
                Text = "左摇杆 · 上下拖动", Dock = DockStyle.Fill,
                BackColor = Color.White, Padding = new Padding(16, 23, 16, 12),
                Margin = new Padding(0, 8, 14, 0)
            };
            TableLayoutPanel leftLayout = new TableLayoutPanel {
                Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 6
            };
            leftLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            leftLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 42));
            leftLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 30));
            leftLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 43));
            leftLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 34));
            leftLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 65));
            joystick = new VerticalJoystick { Dock = DockStyle.Fill, Enabled = false };
            joystick.VerticalChanged += delegate {
                command.SetVertical(joystick.Vertical);
                UpdateDisplay();
                // Release/capture loss must send center immediately, not wait for the timer.
                if (command.Vertical == 0) SendCurrentFrame();
            };
            outputLabel = MakeLabel("LV  0", 22, true);
            outputLabel.TextAlign = ContentAlignment.MiddleCenter;
            limitLabel = MakeLabel("输出上限：" + JoystickCommand.DefaultLimit + " / 100", 10, true);
            limitSlider = new TrackBar {
                Dock = DockStyle.Fill, Minimum = 0, Maximum = 100, Value = JoystickCommand.DefaultLimit,
                TickFrequency = 10, SmallChange = 1, LargeChange = 10,
                AccessibleName = "左摇杆输出上限，0 到 100"
            };
            limitSlider.ValueChanged += delegate {
                command.SetLimit(limitSlider.Value);
                CenterAndSend("已调整输出上限，摇杆回中");
            };
            reverseBox = new CheckBox {
                Text = "反向（切换后自动回中）", Dock = DockStyle.Fill,
                AutoSize = false, AccessibleName = "反向左摇杆"
            };
            reverseBox.CheckedChanged += delegate {
                command.SetReverse(reverseBox.Checked);
                CenterAndSend("已切换正负方向，摇杆回中");
            };
            Label leftNote = MakeLabel(
                "松开鼠标、窗口失焦或按空格：速度目标归零。\r\n前后实际方向请现场确认；初始上限为 " +
                JoystickCommand.DefaultLimit + "。\r\n这不是关闭电机，也不关闭平衡控制。", 9, false);
            leftNote.ForeColor = Color.FromArgb(90, 107, 119);
            leftLayout.Controls.Add(joystick, 0, 0);
            leftLayout.Controls.Add(outputLabel, 0, 1);
            leftLayout.Controls.Add(limitLabel, 0, 2);
            leftLayout.Controls.Add(limitSlider, 0, 3);
            leftLayout.Controls.Add(reverseBox, 0, 4);
            leftLayout.Controls.Add(leftNote, 0, 5);
            left.Controls.Add(leftLayout);
            content.Controls.Add(left, 0, 0);

            GroupBox right = new GroupBox {
                Text = "连接与串口输出", Dock = DockStyle.Fill,
                BackColor = Color.White, Padding = new Padding(16, 23, 16, 12),
                Margin = new Padding(0, 8, 0, 0)
            };
            TableLayoutPanel rightLayout = new TableLayoutPanel {
                Dock = DockStyle.Fill, ColumnCount = 1, RowCount = 4
            };
            rightLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 40));
            rightLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 39));
            rightLayout.RowStyles.Add(new RowStyle(SizeType.Absolute, 90));
            rightLayout.RowStyles.Add(new RowStyle(SizeType.Percent, 100));
            statusLabel = MakeLabel("未连接 · 电机状态以小车程序为准", 11, true);
            frameLabel = MakeLabel("[joystick,0,0,0,0]", 11, false);
            frameLabel.Font = new Font("Consolas", 12f);
            frameLabel.BackColor = Color.FromArgb(235, 244, 247);
            frameLabel.Padding = new Padding(8, 0, 0, 0);
            Label instruction = MakeLabel(
                "先在 Windows 蓝牙设置中配对 HC-06，PIN 为 1234。\r\n选择该设备的蓝牙串口，通常为“传出”端口。\r\n不要直接把 USB 的 COM3 当作蓝牙端口。\r\n连接后每 100 ms 发送；接收区显示小车返回的日志。", 9, false);
            instruction.ForeColor = Color.FromArgb(88, 108, 124);
            logBox = new TextBox {
                Dock = DockStyle.Fill, Multiline = true, ReadOnly = true,
                ScrollBars = ScrollBars.Vertical, WordWrap = true,
                Font = new Font("Consolas", 9f), BackColor = Color.FromArgb(26, 39, 50),
                ForeColor = Color.FromArgb(221, 235, 241), BorderStyle = BorderStyle.FixedSingle
            };
            rightLayout.Controls.Add(statusLabel, 0, 0);
            rightLayout.Controls.Add(frameLabel, 0, 1);
            rightLayout.Controls.Add(instruction, 0, 2);
            rightLayout.Controls.Add(logBox, 0, 3);
            right.Controls.Add(rightLayout);
            content.Controls.Add(right, 1, 0);
            root.Controls.Add(content, 0, 2);

            Label footer = MakeLabel(
                "通信中断或窗口无响应时，配套小车程序在 500 ms 未收到有效帧后将速度目标归零；平衡控制继续。", 9, false);
            footer.ForeColor = Color.FromArgb(92, 111, 126);
            root.Controls.Add(footer, 0, 3);
        }

        private void RefreshPorts()
        {
            string previous = portBox.SelectedItem as string;
            portBox.Items.Clear();
            portBox.Items.Add("请选择已配对 HC-06 的蓝牙 COM 端口");
            try
            {
                string[] names = SerialPort.GetPortNames();
                Array.Sort(names, StringComparer.OrdinalIgnoreCase);
                portBox.Items.AddRange(names);
                int previousIndex = previous == null ? -1 : portBox.Items.IndexOf(previous);
                portBox.SelectedIndex = previousIndex > 0 ? previousIndex : 0;
            }
            catch (Exception ex)
            {
                portBox.SelectedIndex = 0;
                AddLog("无法枚举串口：" + ex.Message);
            }
        }

        private void Connect()
        {
            if (portBox.SelectedIndex <= 0)
            {
                AddLog("请先选择 HC-06 对应的蓝牙串口。这里不会自动选择 USB 串口。");
                return;
            }
            CenterAndSend(null);
            SerialPort candidate = null;
            try
            {
                candidate = new SerialPort((string)portBox.SelectedItem, 9600, Parity.None, 8, StopBits.One);
                candidate.Handshake = Handshake.None;
                candidate.DtrEnable = false;
                candidate.RtsEnable = false;
                candidate.ReadTimeout = 20;
                candidate.WriteTimeout = 50;
                candidate.Encoding = Encoding.ASCII;
                candidate.Open();
                // No nonzero value is sent during connection, even after a previous drag.
                candidate.Write(JoystickCommand.MakeFrame(0));
                lock (receiveLock)
                {
                    pendingReceive.Length = 0;
                    pendingError = null;
                    failedPort = null;
                }
                receiveTail = "";
                port = candidate;
                candidate.DataReceived += ReceiveData;
                lastLoggedFrame = JoystickCommand.MakeFrame(0);
                sendTimer.Start();
                AddLog("已连接 " + candidate.PortName + "，9600 baud；已发送回中帧。");
                UpdateDisplay();
            }
            catch (Exception ex)
            {
                if (candidate != null) candidate.Dispose();
                port = null;
                AddLog("连接失败：" + ex.Message);
                UpdateDisplay();
            }
        }

        private void ReceiveData(object sender, SerialDataReceivedEventArgs e)
        {
            SerialPort source = sender as SerialPort;
            if (source == null || source != port) return;
            try
            {
                string received = source.ReadExisting();
                lock (receiveLock)
                {
                    if (source != port) return;
                    pendingReceive.Append(received);
                    if (pendingReceive.Length > 8192)
                        pendingReceive.Remove(0, pendingReceive.Length - 8192);
                }
            }
            catch (Exception ex)
            {
                lock (receiveLock)
                {
                    if (source == port)
                    {
                        pendingError = ex.Message;
                        failedPort = source;
                    }
                }
            }
        }

        private void SendTimerTick(object sender, EventArgs e)
        {
            string received;
            string error;
            SerialPort errorPort;
            lock (receiveLock)
            {
                received = pendingReceive.ToString();
                pendingReceive.Length = 0;
                error = pendingError;
                errorPort = failedPort;
                pendingError = null;
                failedPort = null;
            }
            if (error != null && errorPort == port)
            {
                Disconnect("串口读取失败：" + error, false);
                return;
            }
            if (!string.IsNullOrEmpty(received))
            {
                receiveTail += received.Replace("\r", "");
                int newline;
                while ((newline = receiveTail.IndexOf('\n')) >= 0)
                {
                    string line = receiveTail.Substring(0, newline);
                    receiveTail = receiveTail.Substring(newline + 1);
                    if (line.Length != 0) AddLog("RX " + line);
                }
                if (receiveTail.Length > 2048)
                {
                    AddLog("RX " + receiveTail.Substring(0, 2048) + " …");
                    receiveTail = "";
                }
            }
            SendCurrentFrame();
        }

        private void SendCurrentFrame()
        {
            SerialPort active = port;
            if (active == null || closing) return;
            try
            {
                string frame = command.Frame;
                active.Write(frame);
                if (frame != lastLoggedFrame)
                {
                    AddLog("TX " + frame);
                    lastLoggedFrame = frame;
                }
                UpdateDisplay();
            }
            catch (Exception ex)
            {
                Disconnect("串口发送失败：" + ex.Message +
                    "；已断开，等待小车的 500 ms 通信超时归零。", false);
            }
        }

        private void CenterAndSend(string reason)
        {
            command.Center();
            joystick.ReleaseToCenter();
            UpdateDisplay();
            SendCurrentFrame();
            if (!string.IsNullOrEmpty(reason)) AddLog(reason + "；速度目标为 0。");
        }

        private void Disconnect(string reason, bool sendCenter)
        {
            sendTimer.Stop();
            command.Center();
            SerialPort old = port;
            // Clear first so capture-change events cannot re-enter an unsuccessful write.
            port = null;
            joystick.ReleaseToCenter();
            if (old != null)
            {
                old.DataReceived -= ReceiveData;
                if (sendCenter)
                {
                    try { old.Write(JoystickCommand.MakeFrame(0)); }
                    catch { /* Firmware timeout handles an already broken link. */ }
                }
                try { old.Close(); }
                catch { }
                old.Dispose();
            }
            lock (receiveLock)
            {
                pendingReceive.Length = 0;
                pendingError = null;
                failedPort = null;
            }
            receiveTail = "";
            UpdateDisplay();
            if (!string.IsNullOrEmpty(reason)) AddLog(reason);
        }

        private void UpdateDisplay()
        {
            bool connected = port != null;
            joystick.Enabled = connected;
            portBox.Enabled = !connected;
            connectButton.Text = connected ? "断开" : "连接";
            statusLabel.Text = connected ?
                "已连接 " + port.PortName + " · 9600 baud · 10 Hz" :
                "未连接 · 电机状态以小车程序为准";
            statusLabel.ForeColor = connected ? Color.FromArgb(0, 119, 100) : Color.FromArgb(140, 94, 29);
            outputLabel.Text = "LV  " + command.Output;
            frameLabel.Text = command.Frame;
            limitLabel.Text = "输出上限：" + command.Limit + " / 100";
        }

        private void AddLog(string line)
        {
            if (logBox == null) return;
            if (line.Length > 2200) line = line.Substring(0, 2200) + " …";
            logLines.Enqueue(DateTime.Now.ToString("HH:mm:ss.fff") + "  " + line);
            while (logLines.Count > 180) logLines.Dequeue();
            string combined = string.Join(Environment.NewLine, logLines.ToArray());
            while (combined.Length > 30000 && logLines.Count > 1)
            {
                logLines.Dequeue();
                combined = string.Join(Environment.NewLine, logLines.ToArray());
            }
            logBox.Text = combined;
            logBox.SelectionStart = logBox.TextLength;
            logBox.ScrollToCaret();
        }

        protected override void OnDeactivate(EventArgs e)
        {
            base.OnDeactivate(e);
            if (!closing) CenterAndSend(null);
        }

        protected override bool ShowWithoutActivation
        {
            get { return offscreenRendering || base.ShowWithoutActivation; }
        }

        internal void RenderOffscreen(string path)
        {
            // WinForms skips child painting for a hidden parent. Lay it out in an
            // entirely transparent, non-activating, off-desktop window instead.
            offscreenRendering = true;
            ShowInTaskbar = false;
            StartPosition = FormStartPosition.Manual;
            Location = new Point(-32000, -32000);
            Opacity = 0;
            Show();
            PerformLayout();
            using (Bitmap bitmap = new Bitmap(Width, Height))
            {
                DrawToBitmap(bitmap, new Rectangle(Point.Empty, bitmap.Size));
                bitmap.Save(path, System.Drawing.Imaging.ImageFormat.Png);
            }
            Hide();
        }

        protected override void OnKeyDown(KeyEventArgs e)
        {
            if (e.KeyCode == Keys.Space)
            {
                CenterAndSend("空格回中");
                e.Handled = true;
                e.SuppressKeyPress = true;
            }
            base.OnKeyDown(e);
        }

        protected override void OnFormClosing(FormClosingEventArgs e)
        {
            closing = true;
            Disconnect(null, true);
            base.OnFormClosing(e);
        }

        protected override void Dispose(bool disposing)
        {
            if (disposing)
            {
                if (port != null) Disconnect(null, true);
                sendTimer.Dispose();
            }
            base.Dispose(disposing);
        }
    }

    internal static class Program
    {
        [STAThread]
        private static int Main(string[] args)
        {
            if (args.Length > 0 && args[0] == "--self-test")
                return SelfTest(args.Length > 1 ? args[1] : null);

            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            if (args.Length == 2 && args[0] == "--render")
            {
                // Creates handles for offscreen rendering only; never opens a serial port.
                using (JoystickForm form = new JoystickForm())
                {
                    form.RenderOffscreen(args[1]);
                }
                return 0;
            }
            if (args.Length != 0) return 2;
            Application.Run(new JoystickForm());
            return 0;
        }

        private static int SelfTest(string reportPath)
        {
            List<string> results = new List<string>();
            try
            {
                JoystickCommand state = new JoystickCommand();
                Check(state.Frame == "[joystick,0,0,0,0]", "initial center", results);
                state.SetVertical(100);
                Check(state.Output == 50, "default maximum 50", results);
                state.SetVertical(-100);
                Check(state.Frame == "[joystick,0,-50,0,0]", "negative LV, other axes zero", results);
                state.SetVertical(500);
                Check(state.Output == 50, "input clamps at 100", results);
                state.SetLimit(100);
                Check(state.Output == 0, "limit change returns to center", results);
                state.SetVertical(50);
                Check(state.Output == 50, "full-range mapping", results);
                state.SetReverse(true);
                Check(state.Output == 0, "reverse change returns to center", results);
                state.SetVertical(100);
                Check(state.Output == -100, "reverse mapping", results);
                state.Center();
                Check(state.Output == 0, "explicit center", results);
                state.SetLimit(0);
                state.SetVertical(100);
                Check(state.Output == 0, "zero maximum always centers command", results);
                Check(JoystickCommand.MakeFrame(200) == "[joystick,0,100,0,0]",
                    "protocol bounds", results);
                for (int limit = 0; limit <= 100; ++limit)
                {
                    state.SetLimit(limit);
                    for (int input = -100; input <= 100; ++input)
                    {
                        state.SetVertical(input);
                        if (Math.Abs(state.Output) > limit)
                            throw new InvalidOperationException("output limit invariant");
                    }
                }
                results.Add("PASS all 20,301 input/limit combinations remain bounded");
                results.Add("PASS no serial port opened; no visible window launched");
                WriteReport(results, reportPath);
                return 0;
            }
            catch (Exception ex)
            {
                results.Add("FAIL " + ex.Message);
                WriteReport(results, reportPath);
                return 1;
            }
        }

        private static void Check(bool condition, string label, List<string> results)
        {
            if (!condition) throw new InvalidOperationException(label);
            results.Add("PASS " + label);
        }

        private static void WriteReport(List<string> lines, string path)
        {
            string text = string.Join(Environment.NewLine, lines.ToArray());
            if (path != null) File.WriteAllText(path, text, new UTF8Encoding(false));
            Console.WriteLine(text);
        }
    }
}
