using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO.Ports;

public class MPU_1 : MonoBehaviour
{
	public string[] receavedGyroPosition = new string[14];
	public ReceiveData rc_1;
    void Start()
    {
        
    }

    void Update()
    {
    	receavedGyroPosition[0] = rc_1.gyroPosition[0];
    	receavedGyroPosition[1] = rc_1.gyroPosition[1];
    	receavedGyroPosition[2] = rc_1.gyroPosition[2];
    	receavedGyroPosition[3] = rc_1.gyroPosition[3];
        receavedGyroPosition[4] = rc_1.gyroPosition[4];
        receavedGyroPosition[5] = rc_1.gyroPosition[5];
        receavedGyroPosition[6] = rc_1.gyroPosition[6];        

    	Quaternion target = new Quaternion(float.Parse(receavedGyroPosition[0]),float.Parse(receavedGyroPosition[1]),float.Parse(receavedGyroPosition[2]),float.Parse(receavedGyroPosition[3]));
        transform.rotation = Quaternion.Slerp(transform.rotation, target, Time.deltaTime * 2.0f);
        transform.position = new Vector3(float.Parse(receavedGyroPosition[4]),float.Parse(receavedGyroPosition[5]),float.Parse(receavedGyroPosition[6]));
    }
}
