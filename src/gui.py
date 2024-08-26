import tkinter as tk
from tkinter import Tk, Scale, Frame, Button


class CovarianceGUI:
    def __init__(self, master: Tk) -> None:
        self.master: Tk = master
        self.master.title("Covariance GUI")

        # Create sliders with larger length
        self.odom_covariance_sliders: list[Scale] = [
            self.create_slider(master, "odom_covariance_1", 0.0),
            self.create_slider(master, "odom_covariance_2", 0.0),
            self.create_slider(master, "odom_covariance_3", 0.0),
            self.create_slider(master, "odom_covariance_4", 0.0),
        ]

        self.imu_covariance_sliders: list[Scale] = [
            self.create_slider(master, "imu_covariance_1", 0.0),
            self.create_slider(master, "imu_covariance_2", 0.0),
        ]

        # Add a button to update values
        self.update_button: Button = tk.Button(
            master, text="Update Values", command=self.update_values
        )
        self.update_button.pack(pady=10)

    def create_slider(self, master: Tk, label: str, initial_value: float) -> Scale:
        frame: Frame = tk.Frame(master)
        frame.pack(padx=10, pady=5)

        tk.Label(frame, text=label).pack(side=tk.LEFT)
        slider: Scale = tk.Scale(
            frame,
            from_=0.0,
            to=1.0,
            resolution=0.005,
            orient=tk.HORIZONTAL,
            length=1000,
        )  # Adjust length here
        slider.set(initial_value)
        slider.pack(side=tk.LEFT)

        return slider

    def update_values(self) -> None:
        odom_covariance: list[float] = [
            slider.get() for slider in self.odom_covariance_sliders
        ]
        imu_covariance: list[float] = [
            slider.get() for slider in self.imu_covariance_sliders
        ]

        print("Updated odom_covariance:", odom_covariance)
        print("Updated imu_covariance:", imu_covariance)


def main() -> None:
    root: Tk = tk.Tk()
    gui: CovarianceGUI = CovarianceGUI(root)
    root.mainloop()


if __name__ == "__main__":
    raise SystemExit(main())
