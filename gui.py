import sys
import csv
from julia import Main
import julia
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QLineEdit, QComboBox, QPushButton, QTextEdit
)


class SimulationApp(QWidget):
    def __init__(self):
        super().__init__()
        self.param_inputs = {}  # Dictionary to store input fields
        self.init_ui()
        self.julia_initialized = False  # Flag to track Julia initialization

    def init_ui(self):
        options = ["Vertical", "Horizontal", "Row"]
        self.layout = QVBoxLayout()  # Make layout an instance variable
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
        
        # Add an output log for messages
        self.output_log = QTextEdit()
        self.output_log.setReadOnly(True)  # Make it read-only for logs
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
        if option_selected == "Vertical":
            params = ["Drone Speed", "Heli Speed", "Drone X Position", "Drone Y Position", "Drone Direction", "Response Distance", "Ascent Rate"]
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
                Main.eval('Pkg.activate(".")')
                Main.include("Particle-Simulation/src/main.jl")
                Main.include("Particle-Simulation/test.jl")
                self.julia_initialized = True
                self.output_log.append("Julia initialized successfully.")
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

    def create_csv(self):
        """Creates a CSV file using the simulation parameters."""
        self.output_log.append("Adding parameters to CSV file.")
        csv_filename = "params.csv"
        csv_data = []

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
                writer.writerow(["param_name", "start", "stop", "steps", "unit"])  # Header
                writer.writerows(csv_data)

            self.output_log.append(f"CSV file created successfully: {csv_filename}")
        except Exception as e:
            self.output_log.append(f"Error creating CSV file: {str(e)}")


if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = SimulationApp()
    window.show()
    sys.exit(app.exec_())
