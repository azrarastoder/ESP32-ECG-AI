Project Description
This project implements a real-time ECG monitoring and analysis system using the ESP32 microcontroller, AD8232 analog front-end, and AI-based interpretation via the OpenAI API.
The system is designed to acquire low-amplitude bioelectrical cardiac signals, process them using digital filtering techniques, and provide automated analysis of the ECG waveform.

System Operation
The AD8232 module captures the electrical activity of the heart and outputs an analog ECG signal. This signal is sampled by the ESP32 at a rate of 200 Hz using its ADC.
To ensure signal quality, several digital filters are implemented:
- **Notch Filter**: Removes power line interference (50/60 Hz noise)
- **Low-pass Filter (~15 Hz cutoff)**: Eliminates high-frequency noise
- **High-pass Filter (~0.5 Hz cutoff)**: Removes baseline drift
These filters are implemented as real-time IIR filters directly on the ESP32.

Data Acquisition and Processing
The system continuously samples ECG data for a defined time window (5 seconds). The filtered signal is then:
1. Converted to voltage values
2. Stored as a time-series dataset
3. Formatted into a compact string representation
To fit API constraints, the dataset is truncated before transmission.

AI-Based Analysis
The processed ECG data is sent to the OpenAI API, where a language model is prompted to analyze the waveform and provide a diagnostic interpretation.
The system configures the AI model with:
- A medical context ("Cardiologist AI Assistant")
- Controlled response variability (low temperature)
- Structured output constraints
The resulting diagnosis is returned and displayed via the serial interface.

Features
- Real-time ECG acquisition using ESP32
- Embedded digital signal processing (DSP)
- Multi-stage filtering pipeline
- Wireless communication via WiFi
- AI-assisted interpretation of biomedical signals

Modes of Operation
The system supports multiple modes via serial commands:
- `read` → Streams raw and filtered ECG signals
- `show` → Same as read (for visualization/debugging)
- `analyze` → Captures data and sends it for AI-based diagnosis

Engineering Significance
This project demonstrates the integration of:
- Embedded systems (ESP32)
- Biomedical signal acquisition
- Digital signal processing (IIR filters)
- IoT communication
- AI-assisted healthcare analysis
It highlights a low-cost, scalable approach to real-time cardiac monitoring and intelligent diagnostics.

Future Improvements
- Implementation of on-device machine learning models
- Improved filtering (adaptive or FIR filters)
- Real-time visualization via web interface
- Data logging and long-term monitoring
