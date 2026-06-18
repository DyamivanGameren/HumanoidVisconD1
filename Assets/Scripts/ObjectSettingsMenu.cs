using UnityEngine;

public class SettingsMenu : MonoBehaviour
{
    public GameObject settingsPanel;

    public void ToggleSettings()
    {
        settingsPanel.SetActive(!settingsPanel.activeSelf);
    }

    public void CloseSettings()
    {
        settingsPanel.SetActive(false);
    }
}