namespace UpdateDemoApp
{
    partial class Form1
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Form1));
            this.lbCount = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.tbHexfile = new System.Windows.Forms.TextBox();
            this.progressBar = new System.Windows.Forms.ProgressBar();
            this.label2 = new System.Windows.Forms.Label();
            this.timer1 = new System.Windows.Forms.Timer(this.components);
            this.label4 = new System.Windows.Forms.Label();
            this.cbEthernet = new System.Windows.Forms.ComboBox();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.btnSendSubnet = new System.Windows.Forms.Button();
            this.button3 = new System.Windows.Forms.Button();
            this.btnCancel = new System.Windows.Forms.Button();
            this.btnBrowse = new System.Windows.Forms.Button();
            this.btnUpload = new System.Windows.Forms.Button();
            this.bntOK = new System.Windows.Forms.Button();
            this.groupBox1.SuspendLayout();
            this.SuspendLayout();
            // 
            // lbCount
            // 
            this.lbCount.AutoSize = true;
            this.lbCount.Location = new System.Drawing.Point(314, 323);
            this.lbCount.Margin = new System.Windows.Forms.Padding(6, 0, 6, 0);
            this.lbCount.Name = "lbCount";
            this.lbCount.Size = new System.Drawing.Size(20, 24);
            this.lbCount.TabIndex = 232;
            this.lbCount.Text = "0";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(247, 323);
            this.label3.Margin = new System.Windows.Forms.Padding(6, 0, 6, 0);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(55, 24);
            this.label3.TabIndex = 231;
            this.label3.Text = "Lines";
            // 
            // tbHexfile
            // 
            this.tbHexfile.Location = new System.Drawing.Point(12, 124);
            this.tbHexfile.Multiline = true;
            this.tbHexfile.Name = "tbHexfile";
            this.tbHexfile.ScrollBars = System.Windows.Forms.ScrollBars.Vertical;
            this.tbHexfile.Size = new System.Drawing.Size(516, 184);
            this.tbHexfile.TabIndex = 228;
            this.tbHexfile.HelpRequested += new System.Windows.Forms.HelpEventHandler(this.tbHexfile_HelpRequested);
            // 
            // progressBar
            // 
            this.progressBar.Location = new System.Drawing.Point(106, 325);
            this.progressBar.Name = "progressBar";
            this.progressBar.Size = new System.Drawing.Size(115, 20);
            this.progressBar.TabIndex = 227;
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(8, 323);
            this.label2.Margin = new System.Windows.Forms.Padding(6, 0, 6, 0);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(89, 24);
            this.label2.TabIndex = 225;
            this.label2.Text = "Firmware";
            // 
            // timer1
            // 
            this.timer1.Interval = 4000;
            this.timer1.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(13, 45);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(76, 24);
            this.label4.TabIndex = 236;
            this.label4.Text = "Local IP";
            // 
            // cbEthernet
            // 
            this.cbEthernet.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.cbEthernet.FormattingEnabled = true;
            this.cbEthernet.Location = new System.Drawing.Point(139, 41);
            this.cbEthernet.Name = "cbEthernet";
            this.cbEthernet.Size = new System.Drawing.Size(157, 32);
            this.cbEthernet.TabIndex = 235;
            this.cbEthernet.SelectedIndexChanged += new System.EventHandler(this.cbEthernet_SelectedIndexChanged);
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.cbEthernet);
            this.groupBox1.Controls.Add(this.btnSendSubnet);
            this.groupBox1.Controls.Add(this.label4);
            this.groupBox1.Controls.Add(this.button3);
            this.groupBox1.Location = new System.Drawing.Point(12, 12);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(516, 106);
            this.groupBox1.TabIndex = 239;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Subnet";
            // 
            // btnSendSubnet
            // 
            this.btnSendSubnet.BackColor = System.Drawing.Color.Transparent;
            this.btnSendSubnet.FlatAppearance.BorderSize = 0;
            this.btnSendSubnet.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnSendSubnet.Image = global::UpdateDemoApp.Properties.Resources.Update4;
            this.btnSendSubnet.Location = new System.Drawing.Point(433, 21);
            this.btnSendSubnet.Name = "btnSendSubnet";
            this.btnSendSubnet.Size = new System.Drawing.Size(72, 72);
            this.btnSendSubnet.TabIndex = 238;
            this.btnSendSubnet.UseVisualStyleBackColor = false;
            this.btnSendSubnet.Click += new System.EventHandler(this.btnSendSubnet_Click);
            this.btnSendSubnet.HelpRequested += new System.Windows.Forms.HelpEventHandler(this.btnSendSubnet_HelpRequested);
            // 
            // button3
            // 
            this.button3.BackColor = System.Drawing.Color.Transparent;
            this.button3.FlatAppearance.BorderSize = 0;
            this.button3.FlatAppearance.MouseDownBackColor = System.Drawing.Color.LightGreen;
            this.button3.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.button3.Font = new System.Drawing.Font("Tahoma", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.button3.Image = global::UpdateDemoApp.Properties.Resources.Update;
            this.button3.Location = new System.Drawing.Point(328, 21);
            this.button3.Name = "button3";
            this.button3.Size = new System.Drawing.Size(72, 72);
            this.button3.TabIndex = 237;
            this.button3.TextAlign = System.Drawing.ContentAlignment.TopLeft;
            this.button3.UseVisualStyleBackColor = false;
            this.button3.Click += new System.EventHandler(this.button3_Click);
            // 
            // btnCancel
            // 
            this.btnCancel.BackColor = System.Drawing.Color.Transparent;
            this.btnCancel.Enabled = false;
            this.btnCancel.FlatAppearance.BorderSize = 0;
            this.btnCancel.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnCancel.Image = global::UpdateDemoApp.Properties.Resources.Cancel64;
            this.btnCancel.Location = new System.Drawing.Point(324, 359);
            this.btnCancel.Name = "btnCancel";
            this.btnCancel.Size = new System.Drawing.Size(76, 72);
            this.btnCancel.TabIndex = 233;
            this.btnCancel.TextAlign = System.Drawing.ContentAlignment.TopLeft;
            this.btnCancel.UseVisualStyleBackColor = false;
            this.btnCancel.Click += new System.EventHandler(this.btnCancel_Click);
            // 
            // btnBrowse
            // 
            this.btnBrowse.BackColor = System.Drawing.Color.Transparent;
            this.btnBrowse.FlatAppearance.BorderSize = 0;
            this.btnBrowse.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnBrowse.Image = global::UpdateDemoApp.Properties.Resources.file;
            this.btnBrowse.ImageAlign = System.Drawing.ContentAlignment.TopCenter;
            this.btnBrowse.Location = new System.Drawing.Point(12, 350);
            this.btnBrowse.Name = "btnBrowse";
            this.btnBrowse.Size = new System.Drawing.Size(76, 72);
            this.btnBrowse.TabIndex = 226;
            this.btnBrowse.TextAlign = System.Drawing.ContentAlignment.TopLeft;
            this.btnBrowse.UseVisualStyleBackColor = false;
            this.btnBrowse.Click += new System.EventHandler(this.btnBrowse_Click);
            this.btnBrowse.HelpRequested += new System.Windows.Forms.HelpEventHandler(this.btnBrowse_HelpRequested);
            // 
            // btnUpload
            // 
            this.btnUpload.BackColor = System.Drawing.Color.Transparent;
            this.btnUpload.FlatAppearance.BorderSize = 0;
            this.btnUpload.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.btnUpload.Image = global::UpdateDemoApp.Properties.Resources.flash;
            this.btnUpload.Location = new System.Drawing.Point(145, 350);
            this.btnUpload.Name = "btnUpload";
            this.btnUpload.Size = new System.Drawing.Size(76, 72);
            this.btnUpload.TabIndex = 222;
            this.btnUpload.TextAlign = System.Drawing.ContentAlignment.TopCenter;
            this.btnUpload.UseVisualStyleBackColor = false;
            this.btnUpload.Click += new System.EventHandler(this.btnUpload_Click);
            this.btnUpload.HelpRequested += new System.Windows.Forms.HelpEventHandler(this.btnUpload_HelpRequested);
            // 
            // bntOK
            // 
            this.bntOK.BackColor = System.Drawing.Color.Transparent;
            this.bntOK.FlatAppearance.BorderSize = 0;
            this.bntOK.FlatStyle = System.Windows.Forms.FlatStyle.Flat;
            this.bntOK.Image = global::UpdateDemoApp.Properties.Resources.bntOK_Image;
            this.bntOK.Location = new System.Drawing.Point(428, 359);
            this.bntOK.Name = "bntOK";
            this.bntOK.Size = new System.Drawing.Size(76, 72);
            this.bntOK.TabIndex = 223;
            this.bntOK.TextAlign = System.Drawing.ContentAlignment.TopLeft;
            this.bntOK.UseVisualStyleBackColor = false;
            this.bntOK.Click += new System.EventHandler(this.bntOK_Click);
            // 
            // Form1
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(11F, 24F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(542, 440);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.btnCancel);
            this.Controls.Add(this.lbCount);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.btnBrowse);
            this.Controls.Add(this.btnUpload);
            this.Controls.Add(this.bntOK);
            this.Controls.Add(this.tbHexfile);
            this.Controls.Add(this.progressBar);
            this.Controls.Add(this.label2);
            this.Font = new System.Drawing.Font("Microsoft Sans Serif", 14.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Margin = new System.Windows.Forms.Padding(6);
            this.MaximizeBox = false;
            this.MinimizeBox = false;
            this.Name = "Form1";
            this.Text = "Firmware Upload";
            this.FormClosed += new System.Windows.Forms.FormClosedEventHandler(this.frmFWTeensyNetwork_FormClosed);
            this.Load += new System.EventHandler(this.frmFWTeensyNetwork_Load);
            this.groupBox1.ResumeLayout(false);
            this.groupBox1.PerformLayout();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion
        private System.Windows.Forms.Button btnCancel;
        private System.Windows.Forms.Label lbCount;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Button btnBrowse;
        private System.Windows.Forms.Button btnUpload;
        private System.Windows.Forms.Button bntOK;
        private System.Windows.Forms.TextBox tbHexfile;
        private System.Windows.Forms.ProgressBar progressBar;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Timer timer1;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.ComboBox cbEthernet;
        private System.Windows.Forms.Button button3;
        private System.Windows.Forms.Button btnSendSubnet;
        private System.Windows.Forms.GroupBox groupBox1;
    }
}

