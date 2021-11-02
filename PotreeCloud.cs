using Fusee.Math.Core;
using Newtonsoft.Json;
using System;
using System.Collections.Generic;
using System.IO;

namespace PotreeSharp
{
    public class PotreeCloud : IDisposable
    {
        private readonly string path;
        private PotreeMetadata metadata;
        public PotreeHierarchy Data;

        private BinaryReader binaryReader;

        public PotreeCloud(string path)
        {
            this.path = path;

            // Loading metadata
            metadata = JsonConvert.DeserializeObject<PotreeMetadata>(File.ReadAllText(path + @"//metadata.json"));

            int pointSize = 0;

            if (metadata != null)
            {
                foreach (var metaAttributeItem in metadata.Attributes)
                {
                    pointSize += metaAttributeItem.Size;
                }

                metadata.PointSize = pointSize;
            }

            // BinaryReader
            binaryReader = new BinaryReader(File.OpenRead(path + @"//octree.bin"));

            // Loading hierarchy
            Data = LoadHierarchy();
        }

        private PotreeHierarchy LoadHierarchy()
        {
            var firstChunkSize = metadata.Hierarchy.FirstChunkSize;
            var stepSize = metadata.Hierarchy.StepSize;
            var depth = metadata.Hierarchy.Depth;

            var data = File.ReadAllBytes(path + @"//hierarchy.bin");

            PotreeNode root = new PotreeNode()
            {
                Name = "r",
                Aabb = new AABBd(metadata.BoundingBox.Min, metadata.BoundingBox.Max)
            };

            var hierarchy = new PotreeHierarchy();

            long offset = 0;
            LoadHierarchyRecursive(ref root, ref data, offset, firstChunkSize);

            hierarchy.Nodes = new();
            root.Traverse(n => hierarchy.Nodes.Add(n));

            hierarchy.TreeRoot = root;

            return hierarchy;
        }

        private void LoadHierarchyRecursive(ref PotreeNode root, ref byte[] data, long offset, long size)
        {
            int bytesPerNode = 22;
            int numNodes = (int)(size / bytesPerNode);

            var nodes = new List<PotreeNode>(numNodes)
            {
                root
            };

            for (int i = 0; i < numNodes; i++)
            {
                var currentNode = nodes[i];
                if (currentNode == null)
                    currentNode = new PotreeNode();

                ulong offsetNode = (ulong)offset + (ulong)(i * bytesPerNode);

                var nodeType = data[offsetNode + 0];
                int childMask = BitConverter.ToInt32(data, (int)offsetNode + 1);
                var numPoints = BitConverter.ToUInt32(data, (int)offsetNode + 2);
                var byteOffset = BitConverter.ToInt64(data, (int)offsetNode + 6);
                var byteSize = BitConverter.ToInt64(data, (int)offsetNode + 14);

                currentNode.NodeType = (NodeType)nodeType;
                currentNode.NumPoints = numPoints;
                currentNode.ByteOffset = byteOffset;
                currentNode.ByteSize = byteSize;

                if (currentNode.NodeType == NodeType.PROXY)
                {
                    LoadHierarchyRecursive(ref currentNode, ref data, byteOffset, byteSize);
                }
                else
                {
                    for (int childIndex = 0; childIndex < 8; childIndex++)
                    {
                        bool childExists = ((1 << childIndex) & childMask) != 0;

                        if (!childExists)
                        {
                            continue;
                        }

                        string childName = currentNode.Name + childIndex.ToString();

                        PotreeNode child = new PotreeNode();

                        child.Aabb = childAABB(currentNode.Aabb, childIndex);
                        child.Name = childName;
                        currentNode.children[childIndex] = child;
                        child.Parent = currentNode;

                        nodes.Add(child);
                    }
                }
            }

            AABBd childAABB(AABBd aabb, int index) {

                double3 min = aabb.min;
                double3 max = aabb.max;

                double3 size = max - min;

                if ((index & 0b0001) > 0)
                {
                    min.z += size.z / 2;
                }
                else
                {
                    max.z -= size.z / 2;
                }

                if ((index & 0b0010) > 0)
                {
                    min.y += size.y / 2;
                }
                else
                {
                    max.y -= size.y / 2;
                }

                if ((index & 0b0100) > 0)
                {
                    min.x += size.x / 2;
                }
                else
                {
                    max.x -= size.x / 2;
                }

                return new AABBd(min, max);
            }
        }

        public PotreeNode FindNode(string id)
        {
            return Data.Nodes.Find(n => n.Name == id);
        }

        // ID is something like "r64" root->6->4
        public PotreeNode LoadNodeData(string id)
        {
            if (!metadata.Encoding.Contains("DEFAULT"))
                throw new NotImplementedException("Non-default encoding is not supported!");

            var node = FindNode(id);

            if (node == null)
                return null;

            var points = new PotreePoint[node.NumPoints];

            var attributeOffset = 0;

            foreach (var metaitem in metadata.Attributes)
            {
                if (metaitem.Name == "position")
                {
                    for (int i = 0; i < node.NumPoints; i++)
                    {
                        binaryReader.BaseStream.Position = node.ByteOffset + attributeOffset + i * metadata.PointSize;

                        points[i].x = (binaryReader.ReadInt32() * (float)metadata.Scale.x) + (float)metadata.Offset.x;
                        points[i].y = (binaryReader.ReadInt32() * (float)metadata.Scale.y) + (float)metadata.Offset.y;
                        points[i].z = (binaryReader.ReadInt32() * (float)metadata.Scale.z) + (float)metadata.Offset.z;

                        // In js they subtract the min offset for every point,I guess that is just moving the pointcloud to the coordinate origin.
                        // We should do this in usercode
                    }
                }
                else if (metaitem.Name.Contains("rgb"))
                {
                    for (int i = 0; i < node.NumPoints; i++)
                    {
                        binaryReader.BaseStream.Position = node.ByteOffset + attributeOffset + i * metadata.PointSize;

                        UInt16 r = binaryReader.ReadUInt16();
                        UInt16 g = binaryReader.ReadUInt16();
                        UInt16 b = binaryReader.ReadUInt16();

                        points[i].r = (byte)(r > 255 ? r / 256 : r);
                        points[i].g = (byte)(g > 255 ? g / 256 : g);
                        points[i].b = (byte)(b > 255 ? b / 256 : b);

                        if (metaitem.Name.Contains("rgba"))
                        {
                            UInt16 a = binaryReader.ReadUInt16();
                            points[i].a = (byte)(a > 255 ? a / 256 : a);
                        }
                        else
                        {
                            points[i].a = 255;
                        }
                    }
                }
                else if (metaitem.Name.Equals("classification"))
                {
                    for (int i = 0; i < node.NumPoints; i++)
                    {
                        binaryReader.BaseStream.Position = node.ByteOffset + attributeOffset + i + metadata.PointSize;

                        points[i].classification = binaryReader.ReadSByte();
                    }
                }

                attributeOffset += metaitem.Size;
            }


            node.points = points;

            node.IsLoaded = true;

            return node;
        }

        public void UnloadNodeData(string id)
        {
           var node = FindNode(id);

           node.points = null;
           node.IsLoaded = false;

           GC.Collect();
        }

        public void Dispose()
        {
            binaryReader.Dispose();
        }
    }
}
