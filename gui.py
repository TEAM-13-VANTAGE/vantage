import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import csv
from julia import Main
import julia
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import *
import time
import subprocess
import visualize

class SimulationApp(QWidget):
    def __init__(self):
        super().__init__()
        self.param_inputs = {}  # Dictionary to store input fields
        self.init_ui()
        self.julia_initialized = False  # Flag to track Julia initialization
        self.sim_type = ""
        self.model = QStandardItemModel()

    def init_ui(self):
        self.layout = QVBoxLayout()  # Make layout an instance variable

        # Add a button to import simulation parameters
        self.import_button = QPushButton("Import Parameters from File")
        self.import_button.clicked.connect(self.import_params_file)
        self.layout.addWidget(self.import_button)

        options = ["Vertical", "Horizontal", "Row"]
        self.params_options = QComboBox()
        self.params_options.addItems(options)
        self.params_options.currentIndexChanged.connect(self.update_input_fields)  # Connect signal
        self.layout.addWidget(self.params_options)

        # Initialize input fields container
        self.input_fields_container = QVBoxLayout()  # Separate container for input fields
        self.layout.addLayout(self.input_fields_container)  # Add it to the main layout

        # Initialize input fields with the default option
        self.create_input_fields(self.params_options.currentText())

        # Add a button to run or process the inputs
        self.run_button = QPushButton("Run Simulation")
        self.run_button.clicked.connect(self.run_simulation)
        self.layout.addWidget(self.run_button)

        # Add a button to launch the high-fidelity module
        self.hf_button = QPushButton("Launch High-Fidelity Simulation")
        self.hf_button.clicked.connect(self.launch_high_fidelity_simulation)
        self.layout.addWidget(self.hf_button)

        # Add an output log for messages
        self.output_log = QTextEdit()
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

        # Predefined unit options
        unit_options = ["feet", "mph", "degrees", "degrees/s", "bool"]

        for param_name in params:
            row = QHBoxLayout()

            # Add label for the parameter
            row.addWidget(QLabel(param_name))

            # Create input fields for start, stop, and steps
            start_input = QLineEdit()
            start_input.setPlaceholderText("Start")
            stop_input = QLineEdit()
            stop_input.setPlaceholderText("Stop")
            steps_input = QLineEdit()
            steps_input.setPlaceholderText("Steps")

            # Create a dropdown for unit selection
            unit_dropdown = QComboBox()
            unit_dropdown.addItems(unit_options)

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


    def initialize_julia(self):
        """Lazy initialization of Julia when the simulation is first run."""
        if not self.julia_initialized:
            self.output_log.append("Initializing Julia...")
            try:
                julia.Julia(compiled_modules=False)  # Initialize Julia
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
        # self.create_csv()

        # # Run Julia simulation and log the results
        # self.output_log.append("Running simulation...")
        # print("Running simulation...")

        # # Initialize Julia if not already initialized
        # self.initialize_julia()

        # if not self.julia_initialized:
        #     self.output_log.append("Julia initialization failed. Cannot run simulation.")
        #     print("Julia initialization failed. Cannot run simulation.")
        #     return

        # visualize.visualize_results(f"Particle-Simulation/results/round1/headon-{self.sim_type[0]}-results.csv", self.sim_type[0])
        self.display_results(f"Particle-Simulation/results/round1/headon-{self.sim_type[0]}-results.csv")

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
            print('Importing sim row: ',row)
            for i in range(1, len(row)):
                # Checks if the input at this index is a LineEdit or ComboBox, sets it to the CSV value
                match self.param_inputs[row[0]][param_values[i]]:
                    case QLineEdit():
                        self.param_inputs[row[0]][param_values[i]].setText(row[i])
                    case QComboBox():
                        self.param_inputs[row[0]][param_values[i]].setCurrentText(row[i])

    def import_params_file(self):
        """Opens and attempts to import a CSV file"""
        csv_filename = QFileDialog.getOpenFileName(self, "Import Parameters from File", ".", "CSV File (*.csv)")[0]

        try:
            with open(csv_filename, newline='', encoding='utf-8') as csv_file:
                reader = csv.reader(csv_file)
                self.sim_type = next(reader) # get sim type header
                print('Sim type: ', self.sim_type)

                if self.sim_type[0] not in ["Vertical", "Horizontal", "Row"]: # the sim types should be an Enum...
                    csv_file.close()
                    raise ValueError("The CSV file cannot be parsed (missing or invalid simulation type)")

                self.import_params(reader)
                csv_file.close()
        except Exception as e:
            self.output_log.append(f"Error parsing CSV file: {str(e)}")

    def launch_high_fidelity_simulation(self):
        """Launch Gazebo, ArduCopter, and MAVProxy in WSL."""
        try:
            # Start Gazebo with the specified world file in WSL
            self.output_log.append("Launching Drone 0...")
            subprocess.run(['wsl', '-d', 'Ubuntu-20.04', 'bash', '-c', '~/run_program.sh'], check=True)
            self.output_log.append("Gazebo launched successfully.")

            # Wait for Gazebo to settle before launching drones
            self.output_log.append("Waiting 30 seconds before launching drones...")
            time.sleep(30)


        except subprocess.CalledProcessError as e:
            self.output_log.append(f"Error: {str(e)}")
        except Exception as e:
            self.output_log.append(f"An unexpected error occurred: {str(e)}")

    def display_results(self, result_file: str):
        """Displays each row of the results CSV file in a GUI."""
        try:
            # Open the CSV file
            with open(result_file, newline='', encoding='utf-8') as csv_file:
                reader = csv.DictReader(csv_file)
                self.init_model()
                # initialize table_view formatting and options
                self.table_view = QTableView()
                self.table_view.setModel(self.model)
                self.table_view.verticalHeader().setVisible(False)
                self.table_view.horizontalHeader().setDefaultAlignment(Qt.AlignmentFlag.AlignLeft)
                self.table_view.setEditTriggers(QAbstractItemView.EditTrigger.NoEditTriggers)
                self.table_view.setSortingEnabled(True)
                # initialize search box
                self.search_results = QLineEdit("")
                self.search_results.setPlaceholderText("Search the results...")
                self.search_results.setObjectName(u"search_results")
                self.search_results.setAlignment(Qt.AlignmentFlag.AlignLeft)
                self.search_results.textChanged.connect(self.filter_results)

                self.layout.addWidget(self.table_view)
                self.layout.addWidget(self.search_results)

                self.populate_model(reader)
                self.init_proxy_model()
        except Exception as e:
            self.output_log.append(f"Error displaying results: {str(e)}")

    def init_model(self):
        """Adds the appropriate table header columns depending on self.sim_type"""
        self.model.clear()
        header_labels = ["Step", "Contact Level", "Scenario"]

        if self.sim_type[0] == "Vertical":
            header_labels.append("Step")
            header_labels.append("Contact Level")
            header_labels.append("Scenario")
            header_labels.append("Drone Ascent Rate")
            header_labels.append("Drone Direction")
            header_labels.append("Drone Response Distance")
            header_labels.append("Drone Speed")
            header_labels.append("Drone X Pos")
            header_labels.append("Drone Y Pos")
            header_labels.append("Heli Speed")
            header_labels.append("Min Distance")
        elif self.sim_type[0] == "Horizontal":
            header_labels.append("Step")
            header_labels.append("Contact Level")
            header_labels.append("Scenario")
            header_labels.append("Drone Direction")
            header_labels.append("Drone Horizontal Turn Angle")
            header_labels.append("Drone Horizontal Turn Rate")
            header_labels.append("Drone Response Distance")
            header_labels.append("Drone Speed")
            header_labels.append("Drone X Pos")
            header_labels.append("Drone Y Pos")
            header_labels.append("Heli Speed")
            header_labels.append("Drone Min Distance")
        elif self.sim_type[0] == "Row":
            header_labels.append("Step")
            header_labels.append("Contact Level")
            header_labels.append("Scenario")
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
            self.proxy_model.setFilterRegExp(QRegExp(search_text, Qt.CaseSensitivity.CaseInsensitive))
        else:
            self.proxy_model.setFilterRegExp("")

    def populate_model(self, reader):
        """Adds data from the CSV reader to self.model"""
        for row in reader:
            row_data = []
            for value in row.values():
                result = QStandardItem(value)
                row_data.append(result)
            self.model.appendRow(row_data)

    def init_proxy_model(self):
        """Initializes the proxy model for sorting and searching the table"""
        self.proxy_model = QSortFilterProxyModel(self)
        self.proxy_model.setSourceModel(self.model)
        self.proxy_model.setFilterKeyColumn(-1)
        self.table_view.setModel(self.proxy_model)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SimulationApp()
    window.show()
    sys.exit(app.exec_())
