using System;
using System.Collections.Generic;
using System.Text;
using Windows.UI.ViewManagement;
using Microsoft.Win32;
using System.Windows;

namespace CAPlanetSwarmSample.WPF.Services
{
    public class CommonDialogService
    {
        public static string? OpenSwarmInputFileDialog()
        {
            var dialog = new OpenFileDialog
            {
                DefaultExt = ".json",
                Filter = $"Swarm Input Data File|*.json",
                Multiselect = false
            };
            if (dialog.ShowDialog(Application.Current.MainWindow) == false)
            {
                return null;
            }

            return dialog.FileName;
        }
    }
}
