namespace PotreeSharp
{
    class Program
    {
        static void Main(string[] args)
        {
            using var pt = new PotreeCloud(@"AxisCloud\pointclouds");

            var rootNode = pt.LoadNodeData("r");

            pt.UnloadNodeData("r");
        }
    }
}
