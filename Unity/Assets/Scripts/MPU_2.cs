using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;

public class MPU_2 : MonoBehaviour
{
	public string[] receavedGyroPosition = new string[14];
	public ReceiveData rc_2;
    void Start()
    {
        
    }

    void Update()
    {
    	receavedGyroPosition[0] = rc_2.gyroPosition[7];
    	receavedGyroPosition[1] = rc_2.gyroPosition[8];
    	receavedGyroPosition[2] = rc_2.gyroPosition[9];
    	receavedGyroPosition[3] = rc_2.gyroPosition[10];
        receavedGyroPosition[4] = rc_2.gyroPosition[11];
        receavedGyroPosition[5] = rc_2.gyroPosition[12];
        receavedGyroPosition[6] = rc_2.gyroPosition[13];  

    	Quaternion target = new Quaternion(float.Parse(receavedGyroPosition[0]),float.Parse(receavedGyroPosition[1]),float.Parse(receavedGyroPosition[2]),float.Parse(receavedGyroPosition[3]));
        transform.rotation = Quaternion.Slerp(transform.rotation, target, Time.deltaTime * 2.0f);
        transform.position = new Vector3(float.Parse(receavedGyroPosition[4]),float.Parse(receavedGyroPosition[5]),float.Parse(receavedGyroPosition[6]));
    }
}
