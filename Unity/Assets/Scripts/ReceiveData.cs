using UnityEngine;
using System.Collections;
using System.IO.Ports;

public class ReceiveData : MonoBehaviour {

	public string[] gyroPosition = new string[14];
	
	SerialPort stream = new SerialPort("/dev/cu.usbmodem14201", 115200); // Defines the serial port and the speed

	void Start () {
		stream.Open(); // Opens the serial port
	}

	void Update () {
		//string[] gyroPosition = new string[14];
		print (stream.ReadLine ());
		string streamInput = stream.ReadLine(); // Reads the data from the arduino card
		gyroPosition = streamInput.Split(','); // Splits the data from the arduino card so that we have values to work with

	}


}