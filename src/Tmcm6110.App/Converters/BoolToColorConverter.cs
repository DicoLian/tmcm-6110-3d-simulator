using System.Globalization;
using System.Windows.Data;
using System.Windows.Media;

namespace Tmcm6110.App.Converters;

public class BoolToColorConverter : IValueConverter
{
    public Color TrueColor { get; set; } = Color.FromRgb(0, 230, 118);
    public Color FalseColor { get; set; } = Color.FromRgb(55, 65, 81);

    public object Convert(object value, Type targetType, object parameter, CultureInfo culture)
    {
        bool flag = value is bool b && b;
        return new SolidColorBrush(flag ? TrueColor : FalseColor);
    }

    public object ConvertBack(object value, Type targetType, object parameter, CultureInfo culture) =>
        throw new NotSupportedException();
}
