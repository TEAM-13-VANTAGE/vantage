import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import csv

from julia.api import Julia
Julia(compiled_modules=False)

from julia import Main
from PyQt5 import QtWidgets, QtGui, QtCore
import time
import subprocess
import drone_launch
import visualize
import threading
import drone_commands
from decimal import Decimal, ROUND_HALF_UP

class SimulationApp(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.param_inputs = {}  # Dictionary to store input fields
        self.julia_initialized = False  # Flag to track Julia initialization
        self.sim_type = ""
        self.model = QtGui.QStandardItemModel()
        self.high_fidelity_parameters = []
        self.init_ui()

    def init_ui(self):
        """Initializes core UI elements"""
        self.layout = QtWidgets.QVBoxLayout()  # Make layout an instance variable

        # Add a button to import simulation parameters
        self.import_button = QtWidgets.QPushButton("Import Parameters from File")
        self.import_button.clicked.connect(self.import_params_file)
        self.layout.addWidget(self.import_button)

        options = ["Vertical", "Horizontal", "Row"]
        self.params_options = QtWidgets.QComboBox()
        self.params_options.addItems(options)
        self.params_options.currentIndexChanged.connect(self.update_input_fields)  # Connect signal
        self.layout.addWidget(self.params_options)

        # Initialize input fields container
        self.input_fields_container = QtWidgets.QVBoxLayout()  # Separate container for input fields
        self.layout.addLayout(self.input_fields_container)  # Add it to the main layout

        # Initialize input fields with the default option
        self.create_input_fields(self.params_options.currentText())

        # Add a button to run or process the inputs
        self.run_button = QtWidgets.QPushButton("Run Simulation")
        self.run_button.clicked.connect(self.run_simulation)
        self.layout.addWidget(self.run_button)
        
        # Add a button to import existing results
        self.import_results_button = QtWidgets.QPushButton("Import Simulation Results")
        self.import_results_button.clicked.connect(self.import_results_file)
        self.layout.addWidget(self.import_results_button)

        # Add a button to launch the high-fidelity module
        self.hf_button = QtWidgets.QPushButton("Launch High-Fidelity Simulation")
        self.hf_button.clicked.connect(self.launch_high_fidelity_simulation)
        self.layout.addWidget(self.hf_button)

        # Disable if no high-fidelity parameters are set
        if not self.high_fidelity_parameters:
            self.hf_button.setEnabled(False)

        

        # Add an output log for messages
        self.output_log = QtWidgets.QTextEdit()
        self.output_log.setReadOnly(True)  # Make it read-only for logs
        self.output_log.setMaximumHeight(150)
        self.layout.addWidget(self.output_log)

        self.setLayout(self.layout)

    def update_input_fields(self):
        """Clear existing input fields and recreate them based on the selected option."""
        # Clear only the input fields container
        self.clear_layout(self.input_fields_container)

        # Get the selected option and recreate input fields
        option_selected = self.params_options.currentText()
        self.create_input_fields(option_selected)

    def clear_layout(self,layout):
        """Recursively clear all widgets and sub-layouts from a layout."""
        while layout.count():
            item = layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()  # Delete widget
            elif item.layout():
                self.clear_layout(item.layout())  # Recursively clear sub-layouts

    def create_input_fields(self, option_selected):
        """Creates labeled input fields for simulation parameters."""
        # Define parameters based on the selected option
        params = []
        if option_selected == "Vertical":
            params = ["drone_speed", "heli_speed", "drone_x_pos", "drone_y_pos", "drone_direction", "drone_response_distance", "drone_ascent_rate"]
        elif option_selected == "Row":
            params = ["drone_speed", "heli_speed", "drone_x_pos", "drone_y_pos", "drone_direction", "drone_response_distance", "drone_horizontal_turn_rate", "drone_horizontal_turn_angle", "force_right_turn"]
        elif option_selected == "Horizontal":
            params = ["drone_speed", "heli_speed", "drone_x_pos", "drone_y_pos", "drone_direction", "drone_response_distance", "drone_horizontal_turn_rate", "drone_horizontal_turn_angle"]

        self.param_inputs = {}  # Tracks input fields for later access



        for param_name in params:
            row = QtWidgets.QHBoxLayout()

            # Add label for the parameter
            row.addWidget(QtWidgets.QLabel(param_name))

            # Create input fields for start, stop, and steps
            start_input = QtWidgets.QLineEdit()
            start_input.setPlaceholderText("Start")
            stop_input = QtWidgets.QLineEdit()
            stop_input.setPlaceholderText("Stop")
            steps_input = QtWidgets.QLineEdit()
            steps_input.setPlaceholderText("Steps")

            # Create a dropdown for unit selection
            unit_dropdown = QtWidgets.QComboBox()
            unit_dropdown.addItems(self.get_input_units(param_name))

            # Add inputs to the row layout
            row.addWidget(start_input)
            row.addWidget(stop_input)
            row.addWidget(steps_input)
            row.addWidget(unit_dropdown)

            # Store references to the input fields in param_inputs
            self.param_inputs[param_name] = {
                "start": start_input,
                "stop": stop_input,
                "steps": steps_input,
                "unit": unit_dropdown,
            }

            # Add the row to the input fields container
            self.input_fields_container.addLayout(row)

            # Set self.sim_type to the selected type
            self.sim_type = option_selected

    def get_input_units(self, param_name):
        '''Given a param_name, return a list of appropriate unit types'''
        # Predefined unit options
        unit_options = {
            'time': ['seconds', 'minnutes', 'hours'],
            'distance': ['meters', 'kilometers'],
            'angle': ['degrees', 'radians'],
            'rate': ['m/s', 'km/h'],
            'unitless': ['boolean']
        }
        print(param_name)
        if param_name in ['drone_speed', 'heli_speed', 'drone_ascent_rate', 'drone_horizontal_turn_rate']:
            return unit_options.get('rate')
        if param_name in ['drone_x_pos', 'drone_y_pos', 'drone_response_distance']:
            return unit_options.get('distance')
        if param_name in ['drone_direction', 'drone_horizontal_turn_angle']:
            return unit_options.get('angle')
        if param_name in ['force_right_turn']:
            return unit_options.get('unitless')
        else:
            print('param not in list: ', param_name)
            return ['ERROR']
        

    def initialize_julia(self):
        """Lazy initialization of Julia when the simulation is first run."""
        if not self.julia_initialized:
            self.output_log.append("Initializing Julia...")
            try:
                Main.eval('using Pkg')
                Main.eval('Pkg.activate("./Particle-Simulation")')
                Main.eval('Pkg.instantiate()')
                # Main.include("Particle-Simulation/src/main.jl")
                self.julia_initialized = True
                self.output_log.append("Julia initialized successfully.")
                Main.include("Particle-Simulation/experiments/round1-full.jl")
            except Exception as e:
                self.output_log.append(f"Error initializing Julia: {e}")

    def run_simulation(self):
        """Runs the Julia simulation with user-provided parameters."""

        # Create the CSV data
        self.create_csv()

        # Run Julia simulation and log the results
        self.output_log.append("Running simulation...")
        print("Running simulation...")

        # Initialize Julia if not already initialized
        self.initialize_julia()

        if not self.julia_initialized:
            self.output_log.append("Julia initialization failed. Cannot run simulation.")
            print("Julia initialization failed. Cannot run simulation.")
            return

        print('sim type: ', self.sim_type)
        visualize.visualize_results(f"results/round1/headon-{self.sim_type.lower()}-results.csv", self.sim_type)
        self.display_results(f"results/round1/headon-{self.sim_type.lower()}-results.csv")

    def create_csv(self):
        """Creates a CSV file using the simulation parameters."""
        self.output_log.append("Adding parameters to CSV file.")
        csv_filename = "params.csv"
        csv_data = []   
        print('Simulation Type: ', str(self.params_options.currentText()))

        for param_name, inputs in self.param_inputs.items():
            try:
                # Get values from the input fields
                start = float(inputs["start"].text())
                stop = float(inputs["stop"].text())
                steps = int(inputs["steps"].text())
                unit = inputs["unit"].currentText()  # Get selected unit from the dropdown

                # Append to csv_data in the correct format
                csv_data.append([param_name, start, stop, steps, unit])
            except ValueError:
                self.output_log.append(f"Error: Invalid input for {param_name}.")
                return  # Exit the function if any value is invalid

        # Write the CSV file
        try:
            with open(csv_filename, "w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow([str(self.params_options.currentText()),]) # simulation type
                writer.writerow(["param_name", "start", "stop", "steps", "unit"])  # Header
                writer.writerows(csv_data)

            self.output_log.append(f"CSV file created successfully: {csv_filename}")
        except Exception as e:
            self.output_log.append(f"Error creating CSV file: {str(e)}")

    def import_params(self, reader):
        """Populates the GUI with parameters from a CSV file"""
        param_values = next(reader)
        for row in reader:
            if row != '':
                print('Importing sim row: ',row)
                for i in range(1, len(row)):
                    # Checks if the input at this index is a LineEdit or ComboBox, sets it to the CSV value
                    match self.param_inputs[row[0]][param_values[i]]:
                        case QtWidgets.QLineEdit():
                            self.param_inputs[row[0]][param_values[i]].setText(row[i])
                        case QtWidgets.QComboBox():
                            self.param_inputs[row[0]][param_values[i]].setCurrentText(row[i])

    def import_params_file(self):
        """Opens and attempts to import a CSV file"""
        # Open file selection dialog
        csv_filename = QtWidgets.QFileDialog.getOpenFileName(self, "Import Parameters from File", ".", "CSV File (*.csv)")[0]
        try:
            with open(csv_filename, newline='', encoding='utf-8') as csv_file:
                reader = csv.reader(csv_file)
                first_line = next(reader)

                if first_line[0] not in ["Vertical", "Horizontal", "Row"]: # check if the first line is a valid sim type
                    csv_file.close()
                    raise ValueError("The CSV file cannot be parsed (missing or invalid simulation type)")
                
                self.sim_type = first_line[0]
                print('Sim type from file: ', self.sim_type)
                
                self.params_options.setCurrentText(self.sim_type)
                self.update_input_fields()
                self.import_params(reader)
                csv_file.close()
        except Exception as e:
            self.output_log.append(f"Error parsing CSV file: {str(e)}")
            
    def round_to_2(val):
        """Rounds a float to two decimal places."""
        return float(Decimal(str(val)).quantize(Decimal("0.01"), rounding=ROUND_HALF_UP))

    def launch_high_fidelity_simulation(self):
        """Launch Gazebo, ArduCopter, and MAVProxy."""
        try:
            # print("Updating sdf file with drone parameters...")
            # path = os.path.expanduser("~/ardu_ws/install/ardupilot_gz_gazebo/share/ardupilot_gz_gazebo/worlds/iris_runway.sdf")
            # x_base = self.round_to_2(self.high_fidelity_parameters[0][7])
            # y_base = self.round_to_2(self.high_fidelity_parameters[0][8])

            # drone_commands.update_runway_pose_in_sdf(path, x_base, y_base)
            # drone_commands.update_drone_pose_in_sdf(path, x_base - 5, y_base, "iris")
            # drone_commands.update_drone_pose_in_sdf(path, x_base + 5, y_base, "iris_2")

            # self.output_log.append("SDF file updated with drone parameters.")
           
           # Launch Gazebo and ArduCopter
            self.output_log.append("Launching high-fidelity module...")
            subprocess.run(['chmod', '+x', 'run_program.sh'], check=True)
            subprocess.run(['./run_program.sh'], check=True)
            time.sleep(3)
            
            
           # Define threads
            
            drone0_thread = threading.Thread(
                target=drone_launch.init_drone_0,
                args=(self.high_fidelity_parameters[0], self.params_options.currentText())
            )
            
            drone1_thread = threading.Thread(
                target=drone_launch.init_drone_1,
                args=(self.high_fidelity_parameters[0], self.params_options.currentText())
            )

            # Start both simultaneously
            self.output_log.append("Launching drone 0...")
            drone0_thread.start()
            time.sleep(4)  # Ensure some delay before starting the second drone
            self.output_log.append("Launching drone 1...")
            drone1_thread.start()
            self.output_log.append("High-fidelity module launched successfully.")

        except subprocess.CalledProcessError as e:
            self.output_log.append(f"Error: {str(e)}")
        except Exception as e:
            self.output_log.append(f"An unexpected error occurred: {str(e)}")
            
    def import_results_file(self):
        """Opens and displays a results CSV file into the results table."""
        csv_filename = QtWidgets.QFileDialog.getOpenFileName(self, "Import Results File", ".", "CSV File (*.csv)")[0]
        if not csv_filename:
            return  # User canceled

        # Attempt to extract simulation type from filename
        if "horizontal" in csv_filename.lower():
            self.sim_type = "Horizontal"
        elif "vertical" in csv_filename.lower():
            self.sim_type = "Vertical"
        elif "row" in csv_filename.lower():
            self.sim_type = "Row"
        else:
            self.output_log.append("Warning: Could not determine simulation type from filename.")
            return

        try:
            self.display_results(csv_filename)
            self.output_log.append(f"Results loaded successfully from {csv_filename}")
        except Exception as e:
            self.output_log.append(f"Error loading results: {str(e)}")
            

    def display_results(self, result_file: str):
        """Displays each row of the results CSV file in a GUI."""
        try:
            # Open the CSV file
            with open(result_file, newline='', encoding='utf-8') as csv_file:
                reader = csv.DictReader(csv_file)
                self.init_model()
                # initialize table_view formatting and options
                self.table_view = QtWidgets.QTableView()
                self.table_view.setModel(self.model)
                self.table_view.verticalHeader().setVisible(False)
                self.table_view.horizontalHeader().setDefaultAlignment(QtCore.Qt.AlignmentFlag.AlignLeft)
                self.table_view.setEditTriggers(QtWidgets.QAbstractItemView.EditTrigger.NoEditTriggers)
                self.table_view.setSortingEnabled(True)
                # initialize search box
                self.search_results = QtWidgets.QLineEdit("")
                self.search_results.setPlaceholderText("Search the results...")
                self.search_results.setObjectName(u"search_results")
                self.search_results.setAlignment(QtCore.Qt.AlignmentFlag.AlignLeft)
                self.search_results.textChanged.connect(self.filter_results)

                self.layout.addWidget(self.table_view)
                self.layout.addWidget(self.search_results)

                self.populate_model(reader)
                self.init_proxy_model()
                self.init_radio_buttons()
        except Exception as e:
            self.output_log.append(f"Error displaying results: {str(e)}")

    def init_model(self):
        """Adds the appropriate table header columns depending on self.sim_type"""
        self.model.clear()
        header_labels = ["Selected", "Step", "Contact Level", "Scenario"]
        if self.sim_type == "Vertical":
            header_labels.append("Drone Ascent Rate")
            header_labels.append("Drone Direction")
            header_labels.append("Drone Response Distance")
            header_labels.append("Drone Speed")
            header_labels.append("Drone X Pos")
            header_labels.append("Drone Y Pos")
            header_labels.append("Heli Speed")
            header_labels.append("Min Distance")
        elif self.sim_type == "Horizontal":
            header_labels.append("Drone Direction")
            header_labels.append("Drone Horizontal Turn Angle")
            header_labels.append("Drone Horizontal Turn Rate")
            header_labels.append("Drone Response Distance")
            header_labels.append("Drone Speed")
            header_labels.append("Drone X Pos")
            header_labels.append("Drone Y Pos")
            header_labels.append("Heli Speed")
            header_labels.append("Drone Min Distance")
        elif self.sim_type == "Row":
            header_labels.append("Drone Direction")
            header_labels.append("Drone Horizontal Turn Angle")
            header_labels.append("Drone Horizontal Turn Rate")
            header_labels.append("Drone Response Distance")
            header_labels.append("Drone Speed")
            header_labels.append("Drone X Pos")
            header_labels.append("Drone Y Pos")
            header_labels.append("Force Right Turn")
            header_labels.append("Heli Speed")
            header_labels.append("Min Distance")
        print(header_labels)
        self.model.setHorizontalHeaderLabels(header_labels)

    def filter_results(self):
        """Applies self.search_results to search the table"""
        search_text = self.search_results.text()
        if search_text:
            self.proxy_model.setFilterRegExp(QtCore.QRegExp(search_text, QtCore.Qt.CaseSensitivity.CaseInsensitive))
        else:
            self.proxy_model.setFilterRegExp("")

    def populate_model(self, reader):
        """Adds data from the CSV reader to self.model"""
        for row in reader:
            row_data = []
            row_data.append(QtGui.QStandardItem())
            for value in row.values():
                result = QtGui.QStandardItem(value)
                row_data.append(result)
            self.model.appendRow(row_data)

    def init_radio_buttons(self):
        """Adds one radio button per row in the first column, allows only one selection"""
        self.radio_buttons = []
        self.radio_group = QtWidgets.QButtonGroup(self)  # Ensures only one can be selected
        self.radio_group.setExclusive(True)

        # Inside init_radio_buttons()
        for row in range(self.proxy_model.rowCount()):
            radio = QtWidgets.QRadioButton()
            self.radio_group.addButton(radio, row)

            container = QtWidgets.QWidget()
            layout = QtWidgets.QHBoxLayout(container)
            layout.setContentsMargins(0, 0, 0, 0)
            layout.setAlignment(QtCore.Qt.AlignmentFlag.AlignCenter)
            layout.addWidget(radio)

            proxy_index = self.proxy_model.index(row, 0)
            self.table_view.setIndexWidget(proxy_index, container)
            self.radio_buttons.append(radio)


        # Connect signal to handler
        self.radio_group.buttonToggled.connect(self.handle_radio_selected)




    def handle_radio_selected(self, button, checked):
        """Highlights the selected row and stores it"""
        if not checked:
            return

        proxy_row = self.radio_group.id(button)
        source_row = self.proxy_model.mapToSource(self.proxy_model.index(proxy_row, 0)).row()

        # Clear highlights from all rows
        for row in range(self.model.rowCount()):
            for col in range(self.model.columnCount()):
                item = self.model.item(row, col)
                if item:
                    item.setBackground(QtGui.QBrush(QtCore.Qt.white))

        # Highlight the newly selected row in the source model
        for col in range(0, self.model.columnCount()):  # Skip radio column
            item = self.model.item(source_row, col)
            if item:
                item.setBackground(QtGui.QBrush(QtGui.QColor("#d0f0c0")))  # Light green

        # Save selected row data from the source model
        row_data = []
        for col in range(1, self.model.columnCount()):
            item = self.model.item(source_row, col)
            if item:
                row_data.append(item.text())

        self.high_fidelity_parameters.clear()
        self.high_fidelity_parameters.append(row_data)
        if self.high_fidelity_parameters:
            self.hf_button.setEnabled(True)
        print("Selected row:", row_data)





    def init_proxy_model(self):
        """Initializes the proxy model for sorting and searching the table"""
        self.proxy_model = QtCore.QSortFilterProxyModel(self)
        self.proxy_model.setSourceModel(self.model)
        self.proxy_model.setFilterKeyColumn(-1)
        self.table_view.setModel(self.proxy_model)
        
    
   
if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = SimulationApp()
    window.show()
    sys.exit(app.exec_())