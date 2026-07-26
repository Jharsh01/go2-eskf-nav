#!/usr/bin/env python3
"""Generate the slip-model / ML-fundamentals teaching PDF using fpdf2.

A from-the-basics explanation of how the go2_eskf slip detector works: machine
learning fundamentals, neural-network fundamentals, how the PyTorch network is
built and trained, and how it is deployed as a dependency-free Eigen MLP in the
ESKF. ASCII-only text (fpdf core fonts are latin-1)."""
from fpdf import FPDF
from pathlib import Path


class Doc(FPDF):
    def header(self):
        if self.page_no() == 1:
            return
        self.set_font("Helvetica", "I", 9)
        self.set_text_color(120, 120, 120)
        self.cell(0, 6, "Slip Model & ML Fundamentals - go2_eskf", align="L")
        self.cell(0, 6, f"Page {self.page_no()}", align="R")
        self.ln(8)
        self.set_text_color(0, 0, 0)

    def footer(self):
        pass

    def _reset_x(self):
        self.set_x(self.l_margin)

    def h1(self, text):
        self.set_font("Helvetica", "B", 18)
        self.set_text_color(20, 60, 130)
        if self.get_y() > 235:
            self.add_page()
        self.ln(2)
        self._reset_x()
        self.multi_cell(0, 9, text)
        self.set_text_color(0, 0, 0)
        self.ln(2)

    def h2(self, text):
        self.set_font("Helvetica", "B", 14)
        self.set_text_color(20, 60, 130)
        if self.get_y() > 248:
            self.add_page()
        self.ln(3)
        self._reset_x()
        self.multi_cell(0, 7, text)
        self.set_text_color(0, 0, 0)
        self.ln(1)

    def h3(self, text):
        self.set_font("Helvetica", "B", 12)
        self.set_text_color(60, 60, 60)
        if self.get_y() > 250:
            self.add_page()
        self.ln(2)
        self._reset_x()
        self.multi_cell(0, 6, text)
        self.set_text_color(0, 0, 0)

    def para(self, text):
        self.set_font("Helvetica", "", 10.5)
        self._reset_x()
        self.multi_cell(0, 5.2, text)
        self.ln(1.5)

    def code(self, text):
        self.set_font("Courier", "", 9)
        self.set_fill_color(245, 245, 248)
        self.set_draw_color(220, 220, 230)
        for line in text.split("\n"):
            if len(line) > 95:
                line = line[:92] + "..."
            self._reset_x()
            self.cell(0, 4.8, " " + line, border=0, fill=True)
            self.ln(4.8)
        self.set_font("Helvetica", "", 10.5)
        self.ln(2)

    def bullet(self, text):
        self.set_font("Helvetica", "", 10.5)
        self._reset_x()
        self.cell(5)
        self.cell(4, 5, "-")
        self.multi_cell(0, 5, text)

    def table(self, headers, rows, col_widths):
        self.set_font("Helvetica", "B", 9.5)
        self.set_fill_color(235, 240, 250)
        if self.get_y() > 245:
            self.add_page()
        for h, w in zip(headers, col_widths):
            self.cell(w, 6, h, border=1, fill=True)
        self.ln()
        self.set_font("Helvetica", "", 9.5)
        for row in rows:
            heights = []
            for txt, w in zip(row, col_widths):
                lines = self.multi_cell(w, 4.5, txt, dry_run=True, output="LINES")
                heights.append(max(1, len(lines)) * 4.5)
            row_h = max(heights)
            if self.get_y() + row_h > 282:
                self.add_page()
            y0 = self.get_y()
            x0 = self.get_x()
            for txt, w in zip(row, col_widths):
                self.multi_cell(w, 4.5, txt, border=1, max_line_height=4.5)
                self.set_xy(x0 + w, y0)
                x0 += w
            self.set_y(y0 + row_h)
        self.ln(2)


def main():
    pdf = Doc(format="A4", unit="mm")
    pdf.set_auto_page_break(auto=True, margin=15)
    pdf.set_margins(left=18, top=15, right=18)
    pdf.add_page()

    # ---- Title page
    pdf.set_font("Helvetica", "B", 22)
    pdf.set_text_color(20, 60, 130)
    pdf.ln(38)
    pdf.cell(0, 12, "The Slip Model, From First Principles", align="L")
    pdf.ln(14)
    pdf.set_font("Helvetica", "", 14)
    pdf.set_text_color(60, 60, 60)
    pdf.cell(0, 8, "Machine learning and neural networks, applied", align="L")
    pdf.ln(9)
    pdf.cell(0, 8, "to slip-adaptive state estimation in go2_eskf", align="L")
    pdf.ln(12)
    pdf.set_font("Helvetica", "I", 11)
    pdf.set_text_color(80, 80, 80)
    pdf.set_x(pdf.l_margin)
    pdf.cell(0, 6, "Package: go2_eskf  |  ROS 2 Jazzy  |  PyTorch + Eigen")
    pdf.ln(7)
    pdf.set_x(pdf.l_margin)
    pdf.cell(0, 6, "A self-contained tutorial: no prior ML background assumed.")
    pdf.ln(7)
    pdf.set_text_color(0, 0, 0)

    pdf.add_page()

    # ---- 0. How to read this
    pdf.h1("0. What this document is")
    pdf.para(
        "The go2_eskf localization filter uses a small neural network to decide, "
        "moment to moment, how much to trust the robot's leg odometry. This "
        "document explains that network from the ground up. It assumes no machine "
        "learning background. We start with what 'learning from data' even means, "
        "build up to neural networks, show exactly how the network is written and "
        "trained in PyTorch, and finish with how it is deployed inside the "
        "estimator with no machine-learning runtime on the robot.")
    pdf.para("Reading path:")
    pdf.bullet("Section 1 - the problem in robotics terms (why we need this at all).")
    pdf.bullet("Section 2 - machine learning fundamentals (models, data, loss, training).")
    pdf.bullet("Section 3 - neural network fundamentals (neurons, layers, activations).")
    pdf.bullet("Section 4 - how PyTorch builds and trains the network in this repo.")
    pdf.bullet("Section 5 - the slip model specifically (features, architecture, output).")
    pdf.bullet("Section 6 - deployment: training in Python, inference in C++.")
    pdf.bullet("Section 7 - run it yourself. Section 8 - limits. Appendix - glossary.")

    # ---- 1. The problem
    pdf.h1("1. The problem, in robotics terms")
    pdf.para(
        "The Unitree Go2 is a legged robot. To know where it is, the filter fuses "
        "an IMU (inertial sensor), GPS, and 'leg odometry'. Leg odometry estimates "
        "the body's velocity from the leg joint angles, using one key assumption: "
        "a foot that is planted on the ground does not move in the world. If you "
        "know a foot is stationary and you know the leg geometry, the motion of the "
        "joints tells you how the body moved relative to that foot.")
    pdf.para(
        "That assumption fails the instant a foot SLIPS. The joints still report "
        "motion, so leg odometry confidently reports a body velocity - but it is "
        "wrong. A standard estimator that always trusts leg odometry by a fixed "
        "amount will absorb that wrong measurement and corrupt its position "
        "estimate.")
    pdf.para(
        "The fix we want: detect slip from sensor signals as it happens, and during "
        "slip, tell the estimator to trust leg odometry LESS and lean on the IMU and "
        "GPS instead. 'Detect slip from signals' is a pattern-recognition task, and "
        "that is exactly what machine learning is good at. The rest of this document "
        "is how we turn that idea into working code.")

    # ---- 2. ML fundamentals
    pdf.h1("2. Machine learning fundamentals")
    pdf.para(
        "Machine learning is a way to build a function when you cannot easily write "
        "the rules by hand, but you DO have examples of inputs paired with the "
        "right outputs. Instead of programming the rule, you choose a flexible "
        "function with adjustable knobs (parameters) and let an algorithm tune the "
        "knobs until the function reproduces your examples. Then you hope it also "
        "works on new inputs it never saw.")

    pdf.h2("2.1  Models, parameters, and prediction")
    pdf.para(
        "A 'model' is just a parameterized function y = f(x; theta). Here x is the "
        "input (a list of numbers called FEATURES), y is the output (the "
        "PREDICTION), and theta is the set of internal numbers (the PARAMETERS, also "
        "called weights) that the learning algorithm will adjust. Different values "
        "of theta give different functions. Learning means searching for the theta "
        "that makes f behave the way the examples say it should.")

    pdf.h2("2.2  Supervised learning: features and labels")
    pdf.para(
        "We use SUPERVISED learning: every training example is a pair (x, y_true), "
        "an input and the correct answer (the LABEL). The input x is a feature "
        "vector - a fixed-length list of numbers describing the situation. For the "
        "slip model, x has 8 numbers (commanded-vs-measured velocity disagreement, "
        "joint speeds, contact fraction, and so on - Section 5.2). The label y_true "
        "is 1.0 if that situation was slipping and 0.0 if it was not.")
    pdf.para(
        "A DATASET is a big table: each row is one example, the first columns are "
        "the features, the last column is the label. Training consumes this table.")

    pdf.h2("2.3  Classification vs regression")
    pdf.para(
        "If the output is a category (slip / no-slip), the task is CLASSIFICATION. "
        "If it is a continuous quantity (e.g. predict a temperature), it is "
        "REGRESSION. The slip model is BINARY classification: two classes. We do "
        "not just want a hard yes/no though - we want a confidence between 0 and 1, "
        "a 'slip score' s, which we will later turn into a smooth dial on how much "
        "to trust leg odometry. A score near 1 means 'almost certainly slipping'.")

    pdf.h2("2.4  The loss function: measuring how wrong we are")
    pdf.para(
        "To tune the parameters we need a single number that says how badly the "
        "model is doing on the training data. That number is the LOSS. Lower is "
        "better; training means making the loss small. For binary classification "
        "with a probability output p in [0,1] and true label y in {0,1}, the "
        "standard loss is BINARY CROSS-ENTROPY (BCE):")
    pdf.code(
        "loss for one example:\n"
        "    L = -[ y * log(p) + (1 - y) * log(1 - p) ]\n"
        "\n"
        "    if y = 1 and p -> 1 :  L -> 0      (confident and correct)\n"
        "    if y = 1 and p -> 0 :  L -> +inf   (confident and WRONG)\n"
        "\n"
        "total loss = average of L over all training examples")
    pdf.para(
        "BCE punishes confident wrong answers very hard and rewards confident right "
        "ones. Minimizing it pushes the predicted probabilities toward the labels.")

    pdf.h2("2.5  Training is just minimizing the loss")
    pdf.para(
        "The loss depends on the parameters theta (through the prediction p). So "
        "'find good parameters' becomes 'find the theta that minimizes the loss' - "
        "an optimization problem. The workhorse method is GRADIENT DESCENT.")
    pdf.para(
        "The GRADIENT of the loss with respect to a parameter tells you which way to "
        "nudge that parameter to make the loss go UP. So we step the opposite way, "
        "downhill, by a small amount called the LEARNING RATE (lr):")
    pdf.code(
        "for each parameter w:\n"
        "    w <- w - lr * (dLoss / dw)\n"
        "\n"
        "repeat for many passes over the data (each pass is an EPOCH)")
    pdf.para(
        "Each step lowers the loss a little. After many steps the parameters settle "
        "near a minimum and the model fits the data. The only real subtlety is "
        "computing dLoss/dw for every parameter efficiently - neural network "
        "libraries do this automatically (Section 4.1).")

    pdf.h2("2.6  Generalization and overfitting")
    pdf.para(
        "The goal is not to memorize the training table; it is to work on NEW data. "
        "A model that nails the training set but fails on new inputs has OVERFIT - "
        "it learned noise and quirks instead of the underlying pattern. The standard "
        "guard is to hold out some labelled data as a VALIDATION set, never train on "
        "it, and watch the loss there. If training loss keeps dropping while "
        "validation loss rises, you are overfitting. Smaller models, more data, and "
        "regularization all help. The slip model is deliberately small (a few "
        "hundred parameters), which makes overfitting less likely.")

    pdf.h2("2.7  Feature scaling (standardization)")
    pdf.para(
        "Features often live on wildly different scales: a velocity might be 0.3 "
        "(m/s) while a joint speed is 4.0 (rad/s). Gradient descent works much "
        "better when all inputs are on a comparable scale. So we STANDARDIZE each "
        "feature: subtract its mean and divide by its standard deviation, computed "
        "over the training set, so every feature ends up roughly zero-mean and "
        "unit-variance:")
    pdf.code("x_scaled = (x - mean) / std")
    pdf.para(
        "Crucially, the SAME mean and std must be applied at prediction time. In "
        "this project those numbers are saved inside the model file and applied "
        "automatically at inference, so deployment needs nothing extra (Section 6.1).")

    # ---- 3. NN fundamentals
    pdf.h1("3. Neural network fundamentals")
    pdf.para(
        "A neural network is a particular, very flexible family of parameterized "
        "functions. It is built by stacking simple units so the whole can represent "
        "complicated input-output relationships. You still train it with the loss + "
        "gradient descent recipe from Section 2.")

    pdf.h2("3.1  The neuron: a weighted sum plus a bias")
    pdf.para(
        "The basic unit takes several inputs, multiplies each by a weight, adds them "
        "up, adds a constant bias, and passes the result through an ACTIVATION "
        "function g:")
    pdf.code(
        "inputs:  x1, x2, ..., xn\n"
        "weights: w1, w2, ..., wn   bias: b\n"
        "\n"
        "z = w1*x1 + w2*x2 + ... + wn*xn + b      (a weighted sum)\n"
        "output = g(z)                            (activation applied)")
    pdf.para(
        "The weights and bias are the parameters that learning adjusts. One neuron "
        "alone can only draw a straight-line (linear) boundary. The power comes from "
        "many neurons plus nonlinear activations.")

    pdf.h2("3.2  Activation functions, and why nonlinearity matters")
    pdf.para(
        "If you stack purely linear neurons, the whole stack collapses back into a "
        "single linear function - no matter how many layers. To represent curves "
        "and logical combinations ('slip only if joints fast AND body slow AND "
        "contact low'), you need a NONLINEAR activation between layers. Two we use:")
    pdf.code(
        "ReLU (Rectified Linear Unit):  g(z) = max(0, z)\n"
        "    cheap; passes positives, zeros out negatives. The standard\n"
        "    hidden-layer activation - it lets the network bend.\n"
        "\n"
        "Sigmoid:                       g(z) = 1 / (1 + e^(-z))\n"
        "    squashes any real number into (0, 1). Used on the FINAL layer\n"
        "    so the output reads as a probability - here, the slip score s.")
    pdf.para(
        "So a hidden ReLU layer gives the network expressive power, and a final "
        "sigmoid turns the last number into a 0-to-1 confidence.")

    pdf.h2("3.3  Layers and the multilayer perceptron (MLP)")
    pdf.para(
        "Neurons are organized into LAYERS. All neurons in a layer read the same "
        "inputs (the previous layer's outputs) and produce a vector of outputs for "
        "the next layer. A stack of such fully-connected layers is a MULTILAYER "
        "PERCEPTRON (MLP). The middle layers are HIDDEN layers; their width (number "
        "of neurons) controls capacity. The slip model is an MLP with two hidden "
        "layers of 16 neurons each.")

    pdf.h2("3.4  The forward pass in matrix form")
    pdf.para(
        "Computing a layer's outputs for all its neurons at once is a matrix "
        "multiply. If a layer has 'in' inputs and 'out' neurons, its weights form an "
        "out-by-in matrix W and its biases a length-'out' vector b. The whole "
        "forward pass (input to output) is:")
    pdf.code(
        "x0 = standardize(features)             # Section 2.7\n"
        "x1 = ReLU( W1 @ x0 + b1 )              # hidden layer 1\n"
        "x2 = ReLU( W2 @ x1 + b2 )              # hidden layer 2\n"
        "s  = sigmoid( W3 @ x2 + b3 )           # output: slip score in (0,1)\n"
        "\n"
        "(@ means matrix-vector multiply)")
    pdf.para(
        "That is the entire run-time computation of a trained network: a few matrix "
        "multiplies and activations. It is cheap, which is why it can run inside the "
        "estimator at sensor rate. This same sequence is implemented three times in "
        "this project (PyTorch, NumPy, C++) and checked to agree (Section 6).")

    pdf.h2("3.5  Backpropagation (how the gradients are found)")
    pdf.para(
        "Training needs dLoss/dw for every weight. BACKPROPAGATION is the algorithm "
        "that computes them: it runs the forward pass, then applies the calculus "
        "chain rule backward through the layers, reusing intermediate results so the "
        "cost is about the same as one forward pass. You do not implement it by "
        "hand - PyTorch records the operations and differentiates them "
        "automatically (Section 4.1). Conceptually: forward pass to get the loss, "
        "backward pass to get the gradients, then one gradient-descent step.")

    # ---- 4. PyTorch
    pdf.h1("4. How PyTorch builds and trains the network")
    pdf.para(
        "PyTorch is a Python library for building and training neural networks. The "
        "trainer in this repo (scripts/train_slip_model.py) uses it when available, "
        "and falls back to a plain NumPy linear model otherwise so the pipeline "
        "runs on machines without PyTorch.")

    pdf.h2("4.1  Tensors and autograd")
    pdf.para(
        "A TENSOR is PyTorch's array type (like a NumPy array, but it can also run "
        "on a GPU and, importantly, track operations for differentiation). When you "
        "compute with tensors that have requires_grad set, PyTorch builds a graph of "
        "the operations. Calling loss.backward() then walks that graph backward and "
        "fills in every parameter's gradient automatically. This is AUTOGRAD - it is "
        "why you never write backpropagation by hand.")

    pdf.h2("4.2  Defining the model")
    pdf.para(
        "nn.Sequential chains layers in order. nn.Linear(in, out) is a "
        "fully-connected layer (it holds the W matrix and b vector). nn.ReLU is the "
        "activation. The actual definition used here:")
    pdf.code(
        "import torch.nn as nn\n"
        "net = nn.Sequential(\n"
        "    nn.Linear(8, 16), nn.ReLU(),      # 8 features  -> 16 hidden\n"
        "    nn.Linear(16, 16), nn.ReLU(),     # 16          -> 16 hidden\n"
        "    nn.Linear(16, 1),                 # 16          -> 1  (a logit)\n"
        ")")
    pdf.para(
        "Note the final layer outputs ONE raw number with no sigmoid. That raw "
        "pre-sigmoid value is called a LOGIT. We handle the sigmoid in the loss "
        "(next) for numerical stability, and re-attach it at export time so the "
        "deployed model outputs a probability.")

    pdf.h2("4.3  The loss: BCEWithLogitsLoss")
    pdf.para(
        "BCEWithLogitsLoss combines a sigmoid and the binary cross-entropy of "
        "Section 2.4 into one numerically stable operation. It takes the raw logit "
        "and the true label and returns the BCE loss - without us ever explicitly "
        "computing sigmoid during training (which can overflow for large logits).")
    pdf.code("loss_fn = nn.BCEWithLogitsLoss()")

    pdf.h2("4.4  The optimizer and the training loop")
    pdf.para(
        "An OPTIMIZER applies the gradient-descent update of Section 2.5 to all "
        "parameters. Adam is a popular variant that adapts the step size per "
        "parameter and usually converges faster than plain gradient descent. The "
        "training loop is the recipe from Sections 2.5 and 3.5, written out:")
    pdf.code(
        "opt = torch.optim.Adam(net.parameters(), lr=1e-2)\n"
        "for epoch in range(epochs):\n"
        "    opt.zero_grad()              # clear old gradients\n"
        "    logits = net(X)             # forward pass on all examples\n"
        "    loss = loss_fn(logits, y)   # how wrong are we?\n"
        "    loss.backward()             # autograd: fill in all gradients\n"
        "    opt.step()                  # one gradient-descent update")
    pdf.para(
        "zero_grad clears gradients from the previous step (PyTorch accumulates them "
        "by default). forward, backward, step - that triad is the heart of training "
        "every neural network, no matter how large.")

    pdf.h2("4.5  From a trained net to a portable file")
    pdf.para(
        "After training, the knowledge lives in the layers' W matrices and b "
        "vectors. The exporter pulls those numbers out, tags the layers with their "
        "activations (relu, relu, sigmoid - re-attaching the sigmoid that training "
        "kept inside the loss), prepends the standardization mean/std, and writes a "
        "plain text file. From here on, PyTorch is no longer needed - the file fully "
        "describes the function (Section 6.1).")

    # ---- 5. The slip model
    pdf.h1("5. The slip model itself")

    pdf.h2("5.1  What it predicts and why")
    pdf.para(
        "The model maps the current locomotion situation to a slip score s in [0,1]. "
        "s near 0 means the feet are planted and leg odometry is trustworthy; s near "
        "1 means slipping, so leg odometry should be distrusted. This single number "
        "is the bridge between machine learning and the Kalman filter.")

    pdf.h2("5.2  The feature vector (the input x)")
    pdf.para(
        "Eight numbers, assembled every leg-odometry update. The first three are "
        "DISAGREEMENT terms - the core signal, because under slip the commanded and "
        "joint motion continue while the measured body motion and contact collapse. "
        "The rest give context so the model can tell real slip from normal dynamics.")
    pdf.table(
        headers=["#", "Feature", "Why it signals slip"],
        rows=[
            ["0,1", "commanded minus measured body velocity (x, y)",
             "Core signal: commanded motion persists, measured leg-odom velocity diverges."],
            ["2", "commanded minus measured yaw rate",
             "Same idea for turning: command vs IMU-measured rotation."],
            ["3", "leg-odometry speed",
             "Context: slip looks different at low vs high speed."],
            ["4,5", "joint velocity mean and max",
             "Legs keep cycling under slip even when the body is not moving."],
            ["6", "horizontal motion-acceleration magnitude",
             "Slip events cause horizontal jerks the IMU sees."],
            ["7", "fraction of feet in contact",
             "Slip correlates with reduced or irregular foot contact."],
        ],
        col_widths=[12, 70, 92],
    )
    pdf.para(
        "The layout is defined once (SlipFeatures in slip_model.hpp) and mirrored in "
        "Python, so the feature order can never drift between training and inference.")

    pdf.h2("5.3  The architecture")
    pdf.para(
        "A standardization step followed by a 2-hidden-layer MLP, exactly the "
        "forward pass of Section 3.4:")
    pdf.code(
        "features (8)\n"
        "   -> standardize: (x - mean) / std        [stored in the model file]\n"
        "   -> Linear(8 -> 16) -> ReLU\n"
        "   -> Linear(16 -> 16) -> ReLU\n"
        "   -> Linear(16 -> 1) -> Sigmoid\n"
        "   -> slip score s in (0,1)")
    pdf.para(
        "Why an MLP and not a hand-written threshold? Because slip is a nonlinear "
        "combination of conditions. 'Joints fast' is only suspicious when ALSO 'body "
        "slow' and 'contact low'. A single threshold cannot express that conjunction; "
        "a small MLP learns it from data.")

    pdf.h2("5.4  Turning the score into filter behaviour")
    pdf.para(
        "The estimator's leg-odometry update has a measurement-noise covariance "
        "R_leg. Larger R means 'this measurement is noisier', so the filter moves "
        "its estimate less in response to it. We inflate R using the slip score:")
    pdf.code("R_leg  <-  R_base * (1 + lambda * s)^2")
    pdf.para(
        "At s=0 the factor is 1 (leg odometry fully trusted). The square is there "
        "because R is a variance: scaling the standard deviation by (1 + lambda*s) "
        "squares into the variance. lambda is a sensitivity knob - larger lambda "
        "makes the filter abandon leg odometry faster as slip rises. The result is a "
        "smooth, monotonic dial rather than a hard on/off switch.")

    pdf.h2("5.5  Where the training data comes from")
    pdf.para(
        "Ideally: real logs from sim or hardware runs, labelled as slip / no-slip "
        "(pass them with --data). Until those exist, the trainer uses a "
        "physics-inspired SYNTHETIC dataset. It samples a hidden slip state and then "
        "generates feature values consistent with it: under slip, measured body "
        "velocity collapses relative to commanded, contact drops, and acceleration "
        "spikes. This gives the network genuine structure to learn, so the whole "
        "pipeline is runnable before real data is collected. The synthetic data is a "
        "scaffold, not the final word - on-robot quality requires real labelled logs.")

    # ---- 6. Deployment
    pdf.h1("6. Deployment: train in Python, run in C++")
    pdf.para(
        "A robot estimator should not depend on a heavy machine-learning runtime. "
        "So training happens offline in Python/PyTorch, and the trained model is "
        "exported to a tiny text file that a few lines of C++ can evaluate with no "
        "ML library at all - just linear algebra (Eigen).")

    pdf.h2("6.1  The weights file: the contract")
    pdf.para(
        "A plain-text format lists the input dimension, the standardization mean and "
        "std, then each layer's shape, activation, weight matrix, and bias vector. "
        "It is framework-agnostic: both the PyTorch trainer and the NumPy fallback "
        "write it, and both the C++ and NumPy inference paths read it.")
    pdf.code(
        "input_dim 8\n"
        "mean  <8 numbers>\n"
        "std   <8 numbers>\n"
        "layers 3\n"
        "layer 8 16 relu\n"
        "<16 rows of weights, 8 numbers each> <then 16 biases>\n"
        "layer 16 16 relu\n"
        "...\n"
        "layer 16 1 sigmoid\n"
        "...")
    pdf.para(
        "Choosing plain text over a format like ONNX or TorchScript is deliberate: "
        "loading it needs only string-to-number parsing, and evaluating it needs "
        "only matrix multiplies. No libtorch to cross-compile for the robot.")

    pdf.h2("6.2  Inference in C++ with Eigen")
    pdf.para(
        "SlipModel (include/go2_eskf/slip_model.hpp) is header-only and depends only "
        "on Eigen. It parses the file into matrices and runs the forward pass of "
        "Section 3.4. The output feeds the covariance rule of Section 5.4. This is "
        "what actually runs on the robot, inside the estimator node.")

    pdf.h2("6.3  The NumPy twin and cross-validation")
    pdf.para(
        "A line-for-line NumPy copy of the forward pass (scripts/slip_reference.py) "
        "exists alongside the C++. A cross-validation script feeds the same inputs to "
        "both and checks they produce the same outputs - they agree to about 1e-15 "
        "(machine precision). This is the project's standard discipline: two "
        "independent implementations that must match turn the math into its own "
        "regression test. Any bug on either side breaks the match immediately.")

    pdf.h2("6.4  Integration into the estimator node")
    pdf.para(
        "When enabled, the node subscribes to the commanded velocity and joint "
        "states, gathers the IMU-derived features during prediction, assembles the "
        "8-feature vector on each leg-odometry update, runs SlipModel, inflates "
        "R_leg, and publishes the slip score for inspection. If the model file is "
        "missing or fails to load, the node logs a warning and falls back to a fixed "
        "covariance - slip adaptivity can never crash localization.")

    # ---- 7. Run it
    pdf.h1("7. Reproduce the whole pipeline")
    pdf.code(
        "# 1. Train (PyTorch if installed, else NumPy fallback) -> weights file\n"
        "python3 src/go2_eskf/scripts/train_slip_model.py \\\n"
        "        --out src/go2_eskf/config/slip_model.txt\n"
        "\n"
        "# 2. Confirm C++ and NumPy forward passes agree (~1e-15)\n"
        "python3 src/go2_eskf/scripts/cross_validate_slip.py\n"
        "\n"
        "# 3. Run the estimator with the slip model enabled\n"
        "ros2 run go2_eskf go2_eskf_node --ros-args -p use_slip_model:=true \\\n"
        "  -p slip_model_path:=<ws>/install/go2_eskf/share/go2_eskf/config/slip_model.txt\n"
        "\n"
        "# 4. Retrain on REAL labelled logs once you have them\n"
        "python3 src/go2_eskf/scripts/train_slip_model.py --data runs.csv \\\n"
        "        --out src/go2_eskf/config/slip_model.txt")
    pdf.para(
        "The --data CSV has the 8 feature columns followed by a final 'slip' label "
        "column in [0,1]. Everything downstream (export, cross-validation, C++ "
        "inference) is identical whether the model was trained on real or synthetic "
        "data.")

    # ---- 8. Limitations
    pdf.h1("8. Limitations and natural extensions")
    pdf.bullet(
        "The shipped model is trained on synthetic data. The architecture and "
        "pipeline are proven; on-robot accuracy needs real labelled logs (--data).")
    pdf.bullet(
        "The contact-fraction feature is constant (1.0) until a foot-contact source "
        "is wired into the node.")
    pdf.bullet(
        "The covariance inflation is isotropic - it scales the x and y leg-velocity "
        "noise equally. Directional slip (e.g. only sideways) is not distinguished; "
        "a 2x2 extension could.")
    pdf.bullet(
        "The score is computed per update with no memory. A short temporal smoothing "
        "or hysteresis on s would reduce flicker between slip and no-slip.")
    pdf.bullet(
        "No validation split is used during the synthetic-data demo. Add one when "
        "training on real data to watch for overfitting (Section 2.6).")

    # ---- Appendix glossary
    pdf.add_page()
    pdf.h1("Appendix - Glossary")
    pdf.table(
        headers=["Term", "Meaning"],
        rows=[
            ["Feature", "One input number describing the situation; the model reads a fixed-length feature vector."],
            ["Label", "The correct answer for a training example (here 1=slip, 0=no slip)."],
            ["Parameter / weight", "An internal number the training algorithm adjusts to fit the data."],
            ["Model", "A parameterized function y = f(x; theta) mapping features to a prediction."],
            ["Loss", "A single number measuring how wrong the model is; training minimizes it."],
            ["BCE", "Binary cross-entropy, the loss used for two-class probability outputs."],
            ["Gradient descent", "Iteratively nudging parameters opposite the loss gradient to reduce loss."],
            ["Learning rate", "The step size of each gradient-descent update."],
            ["Epoch", "One full pass over the training dataset."],
            ["Overfitting", "Fitting training noise so the model fails on new data."],
            ["Standardization", "Rescaling each feature to zero mean and unit variance."],
            ["Neuron", "A unit computing a weighted sum of inputs plus bias, then an activation."],
            ["Activation", "A nonlinear function (ReLU, sigmoid) applied to a neuron's weighted sum."],
            ["ReLU", "max(0, z); the standard hidden-layer activation."],
            ["Sigmoid", "1/(1+e^-z); squashes a number into (0,1), used for probabilities."],
            ["Logit", "A raw pre-sigmoid score; sigmoid(logit) is a probability."],
            ["MLP", "Multilayer perceptron: a stack of fully-connected layers with activations."],
            ["Forward pass", "Computing the output from the input through all layers."],
            ["Backpropagation", "Chain-rule algorithm that computes the loss gradient for every weight."],
            ["Autograd", "PyTorch feature that records operations and differentiates them automatically."],
            ["Tensor", "PyTorch's array type (supports GPUs and gradient tracking)."],
            ["Optimizer", "Object (e.g. Adam) that applies gradient-descent updates to parameters."],
            ["Covariance (R)", "In a Kalman filter, how noisy a measurement is; larger R = trusted less."],
        ],
        col_widths=[42, 132],
    )

    out = Path("/home/harsh/Desktop/robotics_ws/SLIP_MODEL_EXPLAINED.pdf")
    pdf.output(str(out))
    print(f"Wrote {out}")


if __name__ == "__main__":
    main()
