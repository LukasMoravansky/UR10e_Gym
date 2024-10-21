using System.Collections;
using System.Collections.Generic;
using UnityEngine;


// This script is attached to Main camera and enables free movement when right click (on hold)
// Space - UP
// Ctrl  - DOWN



public class Camera_Movement : MonoBehaviour
{
    public float Sensitivity;
    public float MovementSpeed;


    // Update is called once per frame
    void Update()
    {
        if(Input.GetMouseButton(1)) // When right-click
        {
            Cursor.visible = false;
            Cursor.lockState = CursorLockMode.Locked;
            Movement();
            Rotation();
        }
        else
        {
            Cursor.visible = true;
            Cursor.lockState = CursorLockMode.None;
        }
        
    }

    public void Rotation()
    {
        Vector3 OrientationInput = new Vector3(-Input.GetAxis("Mouse Y"), Input.GetAxis("Mouse X"), 0);
        transform.Rotate(OrientationInput * Sensitivity);
        Vector3 eulerRotation = transform.rotation.eulerAngles;
        transform.rotation = Quaternion.Euler(eulerRotation.x, eulerRotation.y, 0);
    }

    public void Movement()
    {
        float MoveUp;
        Vector3 DirectionInput = new Vector3(Input.GetAxis("Horizontal"), 0f, Input.GetAxis("Vertical"));
        transform.Translate(DirectionInput * MovementSpeed * Time.deltaTime);
       // Moving up
        if (Input.GetKey("space"))
        {
            MoveUp = 1;
        }
        else if(Input.GetKey("left ctrl"))
        {
            MoveUp = -1;
        }
        else 
        {
            MoveUp = 0;
        }
        transform.Translate( 0, MoveUp * Time.deltaTime, 0);

    }
}
